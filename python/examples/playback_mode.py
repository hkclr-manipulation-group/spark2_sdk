from spark2_sdk.utils import get_config_prefix_path
from spark2_sdk import Spark2

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    print("Playback mode example currently mirrors cpp/examples/playback_mode.cpp and is a minimal stub.")
    print("Use Spark2(config_prefix_path) and the playback APIs to implement a full playback example.")

    robot = Spark2(config_prefix_path)
    _ = robot


if __name__ == "__main__":
    main()
