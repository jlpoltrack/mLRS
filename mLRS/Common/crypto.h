//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRYPTO
//*******************************************************
// Based on Monocypher, https://github.com/LoupVaillant/monocypher
/*
SecretKey handling:
- on bind, a key root is exchanged, which is based on bind phrase, tx uid, rx uid, 8 byte random number
  from that a secret static key is generated
- on first connection two things happen
    - a 8 byte random number from a TRNG is exchanged; the exchange is encrypted and
      authenticated with 4 byte nonce and 4 byte mac using the static key; that nonce is seeded
      from the TRNG at power up, as the static key is reused across power cycles
    - a secret session key is generated, which is based on
      the key root data plus the 8-byte random value
- depending on the privacy level, a mac for authentication is 0, 3, or 8 bytes; the nonce is a
  4 byte frame counter, transmitted in full so the receiver can check it without wrap ambiguity
- the two link directions are cryptographically separated, so that they can never produce the
  same keystream even though both start their nonce counter at zero:
    - each side derives two keys from the session (or static) key, one for the Tx->Rx direction and
      one for the Rx->Tx direction; a side encrypts with the key of its own direction and decrypts
      with the key of the peer's direction
    - in addition the direction is tagged in a nonce byte which is not transmitted
- replay attacks are prevented by requiring the nonce to strictly increase; this is done after the
  mac has been verified, so that a forged frame cannot push the counter forward
- privacy levels
    off: nothing
    level 1: only encryption                      (4 bytes nonce, no authentication, no replay attack prevention)
    level 2: encryption + authentication          (4 bytes nonce, 3 bytes mac, replay attack prevention)
    level 3: stronger encryption + authentication (4 bytes nonce, 8 bytes mac, replay attack prevention)
*/
//*******************************************************
#ifndef CRYPTO_H
#define CRYPTO_H
#pragma once


#include <inttypes.h>


typedef enum {
    CRYPTO_ROLE_TX = 0, // sends tx frames, receives rx frames
    CRYPTO_ROLE_RX,     // sends rx frames, receives tx frames
} CRYPTO_ROLE_ENUM;


class tCrypto
{
  public:
    void Init(uint8_t role, char* const bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t tx_random, uint32_t static_nonce_seed);
    void SetPrivacyLevel(uint8_t privacy_level);

    void SetSessionKey(uint64_t random); // Tx only
    void GetEncryptedRandom(uint8_t random[16]); // Tx only

    void SetSessionKeyFromEncryptedRandom(uint8_t random[16]); // Rx only
    bool InvalidFrameDecrypted(void); // Rx only
    void Disconnected(void); // Rx only

    uint8_t PrivacyLevel(void) { return _privacy_level; }
    uint16_t NonceLen(void);
    void Encrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len);
    bool Decrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len);

    uint64_t Random(void) { return (_random_valid) ? _random : 0; } // Rx only

  private:
    uint8_t _privacy_level;
    uint8_t _role;

    uint8_t _static[64];
    uint8_t _static_key[32];
    uint32_t _static_nonce_u32;
    uint64_t _random;
    bool _random_valid;

    uint8_t _key_send[32];   // key for the direction this node transmits in
    uint8_t _key_recv[32];   // key for the direction the peer transmits in
    uint8_t _dir_send;       // nonce direction tag of the direction this node transmits in
    uint8_t _dir_recv;       // nonce direction tag of the direction the peer transmits in
    uint32_t _nonce_u32;     // nonce counter of the direction this node transmits in

    uint32_t _nonce_u32_last_received;

    bool _decrypt_ok;

    void _set_direction_keys(uint8_t key[32]);

    void _encrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len);
    bool _decrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len);

    void _make_nonce(uint8_t nonce[12], uint32_t nonce_u32, uint8_t dir);
    void _crypt_it(uint8_t* data, uint16_t len, uint8_t key[32], uint8_t nonce[12]);
    void _mac_it(uint8_t mac[16], uint8_t* const data, uint16_t len, uint8_t key[32], uint8_t nonce[12]);
};


#endif // CRYPTO_H
