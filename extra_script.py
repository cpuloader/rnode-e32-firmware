Import("env")
import os
import configparser

TEMPLATE_CSV = "nvs.csv"
NVS_SIZE     = "0x5000"

def generate_nvs(source, target, env):
    current_env = env['PIOENV']
    print(f"Generating NVS for env: {current_env}")

    config = configparser.ConfigParser()
    if not os.path.exists("secrets.ini"):
        print("File secrets.ini not found!")
        return

    config.read("secrets.ini", encoding="utf-8")

    if not config.has_section(current_env):
        print(f"secrets.ini has no [{current_env}]")
        return

    replacements = {
        "WIFISSID": config.get(current_env, "NVS_WIFISSID", fallback="no"),
        "WIFIPSK":  config.get(current_env, "NVS_WIFIPSK",  fallback="no"),
        "WIFION":   config.get(current_env, "NVS_WIFION",   fallback="1"),
        "BLEON":    config.get(current_env, "NVS_BLEON",    fallback="0")
    }

    with open("nvs.csv", "r", encoding="utf-8") as f:
        content = f.read()

    for key, value in replacements.items():
        content = content.replace(f"{{{{{key}}}}}", value)

    final_csv = os.path.join("build", f"nvs_filled-{current_env}.csv")
    with open(final_csv, "w", encoding="utf-8") as f:
        f.write(content)

    final_bin = os.path.join("build", f"nvs-{current_env}.bin")
    cmd = (
      f'pio pkg exec -p "platformio/framework-espidf" '
      f'"components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py" '
      f'generate "{final_csv}" "{final_bin}" {NVS_SIZE}'
    )
    env.Execute(cmd)

#def before_build(source, target, env):
#    generate_nvs(source, target, env)

#env.AddPreAction("buildprog", before_build)

env.AddCustomTarget("generate_nvs", None, generate_nvs, 
                    title="Generate NVS", 
                    description="Manual NVS creation")
