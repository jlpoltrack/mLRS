Import("env")

def rp_generate_bin(source, target, env):
    elf_path = str(target[0])
    bin_path = elf_path.replace(".elf", ".bin")
    env.Execute(
        env.VerboseAction(
            f"arm-none-eabi-objcopy -O binary {elf_path} {bin_path}",
            f"Generating {bin_path}"))

env.AddPostAction("$BUILD_DIR/firmware.elf", rp_generate_bin)
