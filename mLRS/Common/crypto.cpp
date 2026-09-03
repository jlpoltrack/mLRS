//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRYPTO
//*******************************************************


#include <stdlib.h>
#include <string.h>
#include "crypto.h"
#include "thirdparty/monocypher/src/monocypher.h"


// the nonce counter is transmitted in full, so that the receiver can check it exactly and
// the counter space is 2^32 frames; truncating it would make the wrap ambiguous
#define NONCE_LEN       4
#define MAC_LEN         3
#define LVL3_NONCE_LEN  4
#define LVL3_MAC_LEN    8

// direction separation
// the direction tag is placed in a nonce byte which is never transmitted, so it costs no air bytes
#define DIR_TX_TO_RX        1
#define DIR_RX_TO_TX        2
#define DIR_TAG_POS         11 // must be >= max nonce_len

#define KEY_LABEL_LEN       8
#define KEY_LABEL_TX_TO_RX  "mLRStx2r"
#define KEY_LABEL_RX_TO_TX  "mLRSrx2t"


typedef struct {
    uint8_t nonce_len;
    uint8_t mac_len;
} crypto_level_t;


const crypto_level_t crypto_list[] = {
    { .nonce_len = 0,               .mac_len = 0            }, // nothing
    { .nonce_len = NONCE_LEN,       .mac_len = 0            }, // level 1: only encryption
    { .nonce_len = NONCE_LEN,       .mac_len = MAC_LEN      }, // level 2: encryption + authentication
    { .nonce_len = LVL3_NONCE_LEN,  .mac_len = LVL3_MAC_LEN }, // level 3: stronger encryption + authentication
};


#define PRIVACY_LEVEL_NUM  (sizeof(crypto_list)/sizeof(crypto_level_t))


//-------------------------------------------------------
// Crypto API
//-------------------------------------------------------

void tCrypto::Init(uint8_t role, char* const bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t tx_random, uint32_t static_nonce_seed)
{
    _privacy_level = 0;
    _role = (role == CRYPTO_ROLE_RX) ? CRYPTO_ROLE_RX : CRYPTO_ROLE_TX;

    // a node encrypts in its own direction and decrypts in the peer's direction
    _dir_send = (_role == CRYPTO_ROLE_TX) ? DIR_TX_TO_RX : DIR_RX_TO_TX;
    _dir_recv = (_role == CRYPTO_ROLE_TX) ? DIR_RX_TO_TX : DIR_TX_TO_RX;

    memset(_static, 0, sizeof(_static));
    memcpy(_static,                   "mLRS key",    8); //  8 bytes
    memcpy(_static + 8,               bind_phrase,   6); //  6 bytes
    memcpy(_static + 8 + 6,           tx_uid,       12); // 12 bytes
    memcpy(_static + 8 + 6 + 12,      rx_uid,       12); // 12 bytes
    memcpy(_static + 8 + 6 + 12 +12,  &tx_random,    8); //  8 bytes // sum 46 bytes

    // the static key is reused on each Tx power cycle, so start its nonce at a TRNG seeded value,
    // starting at 0 every time would reuse a (key, nonce) pair for the session random exchange
    _static_nonce_u32 = static_nonce_seed;

    _random = 0;
    _random_valid = false; // session key not yet set

    memset(_key_send, 0, sizeof(_key_send));
    memset(_key_recv, 0, sizeof(_key_recv));
    _nonce_u32 = 0;

    _nonce_u32_last_received = 0;

    _decrypt_ok = true;

    // construct static key
    crypto_blake2b(_static_key, 32, _static, 46);

    // set key to static key to have some default
    _set_direction_keys(_static_key);
}


// derive a key per direction, so that the Tx->Rx and Rx->Tx directions can never
// produce the same keystream even though both nonce counters start at zero
void tCrypto::_set_direction_keys(uint8_t key[32])
{
uint8_t key_source[32 + KEY_LABEL_LEN];

    memcpy(key_source, key, 32);

    memcpy(key_source + 32, (_role == CRYPTO_ROLE_TX) ? KEY_LABEL_TX_TO_RX : KEY_LABEL_RX_TO_TX, KEY_LABEL_LEN);
    crypto_blake2b(_key_send, 32, key_source, 32 + KEY_LABEL_LEN);

    memcpy(key_source + 32, (_role == CRYPTO_ROLE_TX) ? KEY_LABEL_RX_TO_TX : KEY_LABEL_TX_TO_RX, KEY_LABEL_LEN);
    crypto_blake2b(_key_recv, 32, key_source, 32 + KEY_LABEL_LEN);

    crypto_wipe(key_source, sizeof(key_source));
}


void tCrypto::SetPrivacyLevel(uint8_t privacy_level)
{
    if (privacy_level >= PRIVACY_LEVEL_NUM) return;

    _privacy_level = privacy_level;
}


//-- handle session random and session key
// The session random is transmitted encrypted, in the following format:
//  0 ..  7: 8 bytes random
//  8 .. 11: 4 bytes nonce, starts with 0
// 12 .. 15: 4 bytes mac

// Tx: called in init sequence
// Rx: called by SetSessionKeyFromEncryptedRandom() when a FRAME_CMD_GET_RX_SETUPDATA frame is received
void tCrypto::SetSessionKey(uint64_t random)
{
uint8_t key_source[64]; // 46 + 8 = 54
uint8_t key[32];

    if (random == 0 || random == UINT64_MAX) return; // don't accept these, should not happen TODO: what to do if it does?

    if (random == _random) { // same key as before, keep the nonce counters running, restarting them would reuse nonces
        _random_valid = true;
        return;
    }

    _random = random;
    _random_valid = true;

    memcpy(key_source,      _static,  46); // 46 bytes
    memcpy(key_source + 46, &_random,  8); //  8 bytes // sum = 54 bytes

    crypto_blake2b(key, 32, key_source, 54);

    _set_direction_keys(key);

    // a fresh key restarts both nonce counters, in either direction
    _nonce_u32 = 0;
    _nonce_u32_last_received = 0;

    crypto_wipe(key_source, sizeof(key_source));
    crypto_wipe(key, sizeof(key));
}


// only Tx: send along with a FRAME_CMD_GET_RX_SETUPDATA frame
void tCrypto::GetEncryptedRandom(uint8_t random[16])
{
uint8_t nonce_buf[12];
uint8_t poly1305_key[32];
uint8_t mac[16];

    if (!_random_valid) while(1){} // must not happen, must have been set before, just to ensure proper code flow

    memset(nonce_buf, 0, 12);
    memcpy(nonce_buf, &_static_nonce_u32, 4);

    _static_nonce_u32++; // ready it for next use

    crypto_chacha20_ietf(random, (uint8_t*)&_random, 8, _static_key, nonce_buf, 1);

    memcpy(random + 8, nonce_buf, 4); // random[8] ... random[11]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce_buf, 0);
    crypto_poly1305(mac, random, 12, poly1305_key);

    memcpy(random + 12, mac, 4); // random[12] ... random[15]
}


// only Rx: called upon receive of a FRAME_CMD_GET_RX_SETUPDATA frame
void tCrypto::SetSessionKeyFromEncryptedRandom(uint8_t random[16])
{
uint8_t nonce_buf[12];
uint8_t poly1305_key[32];
uint8_t mac[16];
uint64_t rand;

    if (_random_valid) return; // has been set already

    memset(nonce_buf, 0, 12);
    memcpy(nonce_buf, random + 8, 4); // random[8] ... random[11]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce_buf, 0);
    crypto_poly1305(mac, random, 12, poly1305_key);
    for (uint8_t i = 0; i < 4; i++) { if (random[12 + i] != mac[i]) return; } // authentication failed

    crypto_chacha20_ietf((uint8_t*)&rand, random, 8, _static_key, nonce_buf, 1);

    SetSessionKey(rand);
}


// only Rx
bool tCrypto::InvalidFrameDecrypted(void)
{
    if (!_privacy_level) return false;

    bool ok = _decrypt_ok;
    _decrypt_ok = true; // reset it for next use, implies that InvalidFrameDecrypted() is only called once per cycle
    return !ok;
}


// only Rx: called when receiver is disconnected
void tCrypto::Disconnected(void)
{
    // TODO: this needs carefully thinking through.
    // one needs to consider differences between re-powered, reconnected
    // currently: for privacy level >= 2, session key stays always persistent

    if (_privacy_level <= 1) { // accept potentially new session random/session key
        _random_valid = false;
    }
}


//-- API miscellaneous

uint16_t tCrypto::NonceLen(void)
{
    if (!_privacy_level) return 0; // no encryption

    return crypto_list[_privacy_level].nonce_len + crypto_list[_privacy_level].mac_len;
}


void tCrypto::Encrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
    if (!_privacy_level) return; // no encryption

    _encrypt_it(data, len, payload_len);
}


bool tCrypto::Decrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
    if (!_privacy_level) return true; // no encryption

    _decrypt_ok = _decrypt_it(data, len, payload_len);
    return _decrypt_ok;
}


//-------------------------------------------------------
// Encryption handlers
//-------------------------------------------------------

// The data is transmitted encrypted, in the following format:
//   3/4 bytes nonce
//   0/3/8 bytes mac
//   data

void tCrypto::_encrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
uint8_t mac[16];
uint8_t nonce[12];
uint8_t mac_len = crypto_list[_privacy_level].mac_len;
uint8_t nonce_len = crypto_list[_privacy_level].nonce_len;

    // update nonce, tagged with our own direction
    // Note: the counter wraps after 2^32 frames, ca 1.4 years of continuous operation at 100 Hz,
    // rebinding or a Tx power cycle starts a new session key well before that
    _nonce_u32++;
    _make_nonce(nonce, _nonce_u32, _dir_send);

    // encrypt data at data[0]
    _crypt_it(data, len, _key_send, nonce);

    if (mac_len) {
        // MAC = poly1305(nonce || ciphertext)
        _mac_it(mac, data, len, _key_send, nonce);
    }

    // move data to payload + mac_len + nonce_len
    memmove(data + mac_len + nonce_len, data, len); // NOT memcpy(), needs to copy from end towards beginning !!

    // correct payload len for the mac and nonce
    *payload_len += mac_len + nonce_len;

    // copy mac into data
    memcpy(data, mac, mac_len); // data[0] ... data[mac_len-1]

    // copy nonce into data, only the counter bytes are transmitted, the direction tag is implicit
    memcpy(data + mac_len, nonce, nonce_len); // data[mac_len] ... data[mac_len+nonce_len-1]
}


bool tCrypto::_decrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
uint8_t received_mac[LVL3_MAC_LEN];
uint32_t received_nonce_u32;
uint8_t nonce[12];
uint8_t mac[16];
uint8_t mac_len = crypto_list[_privacy_level].mac_len;
uint8_t nonce_len = crypto_list[_privacy_level].nonce_len;

    if (len < mac_len + nonce_len) {
        *payload_len = 0; // TODO: what should we do ?
        return false;
    }

    // get mac
    memcpy(received_mac, data, mac_len); // data[0] ... data[mac_len-1]

    // get nonce, tagged with the peer's direction
    received_nonce_u32 = 0;
    memcpy(&received_nonce_u32, data + mac_len, nonce_len); // data[mac_len] ... data[mac_len+nonce_len-1]
    _make_nonce(nonce, received_nonce_u32, _dir_recv);

    // correct len, payload_len for the mac and nonce
    *payload_len -= mac_len + nonce_len;
    len -= mac_len + nonce_len;

    // move data to data[0]
    memmove(data, data + mac_len + nonce_len, len); // NOT memcpy(), needs to copy from beginning towards end !!

    if (mac_len) {
        // calculate MAC over nonce + payload
        _mac_it(mac, data, len, _key_recv, nonce);

        // comparison of mac_len byte mac, constant time, no early out and no branch per byte
        uint8_t diff = 0;
        for (uint8_t i = 0; i < mac_len; i++) { diff |= (mac[i] ^ received_mac[i]); }

        if (diff) { // authentication failed
            *payload_len = 0; // pretend we didn't got data at all // TODO: what should we do ?
            return false;
        }
    }

    // check nonce, don't accept previously seen nonces, to prevent replay attacks
    // only for privacy levels >= 2, level 1 has no mac, so the nonce is not authenticated and an
    // attacker could push the counter forward at will
    // Note: this must come after the mac check, or a forged frame could advance the counter
    // Note: the peer sends a fresh nonce per frame, an ARQ retransmit of an already accepted payload
    // is dropped by the receive ARQ before it gets here, so a duplicate nonce really is a replay
    if (_privacy_level >= 2) {
        if (received_nonce_u32 <= _nonce_u32_last_received) { // seen before, replay
            *payload_len = 0;
            return false;
        }
        _nonce_u32_last_received = received_nonce_u32;
    }

    // decrypt data at data[0]
    _crypt_it(data, len, _key_recv, nonce);

    return true;
}


//-------------------------------------------------------
// Monocypher interface
//-------------------------------------------------------

// the direction tag sits in a nonce byte beyond the transmitted counter bytes, hence a
// Tx->Rx and a Rx->Tx frame never share a (key, nonce) pair even for equal counter values
void tCrypto::_make_nonce(uint8_t nonce[12], uint32_t nonce_u32, uint8_t dir)
{
    memset(nonce, 0, 12);
    memcpy(nonce, &nonce_u32, 4); // nonce[0] ... nonce[3] = nonce_u32
    nonce[DIR_TAG_POS] = dir;
}


void tCrypto::_crypt_it(uint8_t* data, uint16_t len, uint8_t key[32], uint8_t nonce[12])
{
// Note: the counter does not have to start at 0, one just needs to use
// different counter for each block, so always starting with 1 is fine

    crypto_chacha20_ietf(
        data,     // cipher_text,
        data,     // plain_text, same as cipher = in-place encoding
        len,      // text_size,
        key,      // key[32],
        nonce,    // nonce[12],
        1);       // ctr
}


void tCrypto::_mac_it(uint8_t mac[16], uint8_t* const data, uint16_t len, uint8_t key[32], uint8_t nonce[12])
{
uint8_t poly1305_key[32];
crypto_poly1305_ctx ctx;

// Note: the ChaCha20 keystream of the first block is used as key for poly1305 (which wants 32 bytes key)
// so, we use counter = 0
// it is important that for the data then a different counter is used, so use counter = 1 there

    crypto_chacha20_ietf(
        poly1305_key, // cipher_text,
        NULL,         // plain_text, NULL = returns ChaCha20 keystream
        32,           // text_size,
        key,          // key[32],
        nonce,        // nonce[12],
        0);           // ctr

    crypto_poly1305_init(&ctx, poly1305_key);
    crypto_poly1305_update(&ctx, nonce, 12); // the whole nonce, so the direction tag is authenticated too
    crypto_poly1305_update(&ctx, data, len);
    crypto_poly1305_final(&ctx, mac);

    crypto_wipe(poly1305_key, sizeof(poly1305_key));
}

