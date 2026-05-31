Import("env")
import os
import configparser
import time

def write_nvs(source, target, env):
    current_env = env['PIOENV']
    nvs_bin = os.path.join(env.subst("$PROJECT_DIR"), os.path.join("build", f"nvs-{current_env}.bin"))
    if not os.path.exists(nvs_bin):
        print("File nvs.bin not found")
        return

    platform = env.PioPlatform()
    esptool_py = os.path.join(platform.get_package_dir("tool-esptoolpy"), "esptool.py")
    port = env.get("UPLOAD_PORT")

    #time.sleep(2); # wait before write (if automatic after upload)
    cmd = f'"{env["PYTHONEXE"]}" "{esptool_py}" --chip auto --port {port} --baud 115200 write_flash 0x9000 "{nvs_bin}"'
    env.Execute(cmd)

#env.AddPostAction("upload", write_nvs)

env.AddCustomTarget("write_nvs", None, write_nvs, 
                    title="Write NVS", 
                    description="Manual write NVS")