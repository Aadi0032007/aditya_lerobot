# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 07:13:01 2025

@author: aadi
"""

"""llm agent powered inference with just 1 policy at a time"""

import argparse
import logging
import time
from pathlib import Path
from typing import List
import os

from dotenv import load_dotenv, find_dotenv
import speech_recognition as sr
from langchain_openai import ChatOpenAI
import shutil
import tqdm
from langchain_core.tools import tool
from langchain.agents import create_tool_calling_agent, AgentExecutor

from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
# from safetensors.torch import load_file, save_file
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.populate_dataset import (
    create_lerobot_dataset,
    delete_current_episode,
    init_dataset,
    save_current_episode,
)
from lerobot.common.robot_devices.control_utils import (
    control_loop,
    has_method,
    init_keyboard_listener,
    init_policy,
    log_control_info,
    record_episode,
    reset_environment,
    sanity_check_dataset_name,
    stop_recording,
    warmup_record,
)
from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.robot_devices.robots.utils import Robot
from lerobot.common.robot_devices.utils import busy_wait, safe_disconnect
from lerobot.common.utils.utils import init_hydra_config, init_logging, log_say, none_or_int

########################################################################################
# Control modes
########################################################################################


@safe_disconnect
def calibrate(robot: Robot, arms: list[str] | None):
    # TODO(aliberts): move this code in robots' classes
    if robot.robot_type.startswith("stretch"):
        if not robot.is_connected:
            robot.connect()
        if not robot.is_homed():
            robot.home()
        return

    unknown_arms = [arm_id for arm_id in arms if arm_id not in robot.available_arms]
    available_arms_str = " ".join(robot.available_arms)
    unknown_arms_str = " ".join(unknown_arms)

    if arms is None or len(arms) == 0:
        raise ValueError(
            "No arm provided. Use `--arms` as argument with one or more available arms.\n"
            f"For instance, to recalibrate all arms add: `--arms {available_arms_str}`"
        )

    if len(unknown_arms) > 0:
        raise ValueError(
            f"Unknown arms provided ('{unknown_arms_str}'). Available arms are `{available_arms_str}`."
        )

    for arm_id in arms:
        arm_calib_path = robot.calibration_dir / f"{arm_id}.json"
        if arm_calib_path.exists():
            print(f"Removing '{arm_calib_path}'")
            arm_calib_path.unlink()
        else:
            print(f"Calibration file not found '{arm_calib_path}'")

    if robot.is_connected:
        robot.disconnect()

    # Calling `connect` automatically runs calibration
    # when the calibration file is missing
    robot.connect()
    robot.disconnect()
    print("Calibration is done! You can now teleoperate and record datasets!")

@safe_disconnect
def teleoperate(
    robot: Robot, fps: int | None = None, teleop_time_s: float | None = None, display_cameras: bool = False
):
    control_loop(
        robot,
        control_time_s=teleop_time_s,
        fps=fps,
        teleoperate=True,
        display_cameras=display_cameras,
    )

@safe_disconnect
def llm_agent(
    robot: Robot, 
    chain_path: str | None = None,
    fps: int | None = None, 
    teleop_time_s: float | None = None, 
    display_cameras: bool = False,
    typing: bool = False
):
    import pyttsx3
    import base64
    import cv2

    from langchain.schema import SystemMessage
    from langchain_core.prompts import (
        ChatPromptTemplate,
        HumanMessagePromptTemplate,
        MessagesPlaceholder,
    )


    _ = load_dotenv(find_dotenv())
    api_key = os.getenv("OPENAI_API_KEY")
    

    robot.connect()
    engine = pyttsx3.init()



    models = {
        "reach_the_object":  {"repo_id": "/home/revolabs/aditya/aditya_lerobot/outputs/train/act_koch_reach_the_object/checkpoints/last/pretrained_model", "control_time_s": 10},
         # "policy_2": {"repo_id": "aadi/policy_2", "control_time_s": 10}, 
         # "policy_3":{"repo_id": "aadi/policy_3", "control_time_s": 10}
    }

    global policies 
    policies = {}

    for model_name in models:
        model = models[model_name]
        policy_overrides = ["device=cuda"]
        policy, policy_fps, device, use_amp = init_policy(model["repo_id"], policy_overrides)
        policies[model_name] = ({"policy": policy, "policy_fps": policy_fps, "device": device, "use_amp": use_amp, "control_time_s": model["control_time_s"]})


    @tool(return_direct=True)
    def reach_the_object():
        """Reach the object
        """
        global policies
        do_control_loop(policies["reach_the_object"])

        return "Done"
    
    # @tool(return_direct=True)
    # def policy_2():
        
    #     global policies
    #     do_control_loop(policies["policy_2"])

    #     return "Done"
        
    # @tool(return_direct=True)
    # def policy_3():

    #     global policies
    #     do_control_loop(policies["policy_3"])

    #     return "Done"
    
    @tool(return_direct=True)
    def go_to_home_position(rest_position=rest_position):
        """Go back to home position.
        """
        
        print(rest_position)
        current_pos = robot.follower_arms['main'].read("Present_Position")
        print(current_pos)
        steps = 30
        for i in range(1, steps + 1):
                intermediate_pos = current_pos + (rest_position - current_pos) * (i / steps)
                robot.follower_arms['main'].write("Goal_Position", intermediate_pos)
                time.sleep(0.1) #try busy_wait
                
        return "Done"

    @tool(return_direct=True)
    def describe_area():
        """Describing what I can see.
        """

        llm = ChatOpenAI(temperature=0.1, model=llm_model, api_key=api_key)

        observation = robot.capture_observation()
        image_keys = [key for key in observation if "image" in key]
        for key in image_keys:
            if "phone" in key:
                img = cv2.imread(key, cv2.cvtColor(observation[key].numpy(), cv2.COLOR_RGB2BGR))
        
        
        # cv2.imshow("Image", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        _, encoded_img = cv2.imencode('.png', img) 
        base64_img = base64.b64encode(encoded_img).decode("utf-8")    
        
        mime_type = 'image/png'
        encoded_image_url = f"data:{mime_type};base64,{base64_img}"


        chat_prompt_template = ChatPromptTemplate.from_messages(
            messages=[
                SystemMessage(content='Describe in one phrase what objects you see on the table. Not including robot. Start answer with "I see..."'),
                HumanMessagePromptTemplate.from_template(
                     [{'image_url': "{encoded_image_url}", 'type': 'image_url'}],
                )
            ]
        )

        chain = chat_prompt_template | llm
        res = chain.invoke({"encoded_image_url": encoded_image_url})

        return res.content
        
    def do_control_loop(policy_obj):
        global policies, models
        control_loop(
            robot=robot,
            control_time_s=policy_obj["control_time_s"],
            display_cameras=display_cameras,
            policy=policy_obj["policy"],
            device=policy_obj["device"],
            use_amp=policy_obj["use_amp"],
            fps = policy_obj["policy_fps"],
            teleoperate=False,
        )

    agent_prompt = ChatPromptTemplate.from_messages([
        ("system", "You're a helpful robot-arm assistant. Answer super concise."), 
        ("human", "{input}"), 
        ("placeholder", "{agent_scratchpad}"),
    ])

    llm_model = "gpt-4o-mini"

    llm = ChatOpenAI(temperature=0.1, model=llm_model, api_key=api_key)

    # tools = [reach_the_object, policy_2, policy_3, describe_area]
    tools = [reach_the_object,describe_area,go_to_home_position]

    agent = create_tool_calling_agent(llm, tools, agent_prompt)
    agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)

    r = sr.Recognizer()    

    def listen():
        with sr.Microphone() as source:
            audio = r.listen(source)
            print("Processing...")
        try:
            text = r.recognize_google(audio)
            return text
        except Exception as e:
            print("Error: " + str(e))
            return None


    def generate_response(prompt):
        completions = agent_executor.invoke({"input": prompt, })
        message = completions["output"]
        return message

    while True:
        print("Listening...")
        
        if typing:
            # Chat-based input
            audio_prompt = input("Type your command: ")
        else:
            # Voice-based input
            print("Listening...")
            audio_prompt = listen()
            
            
        if audio_prompt is not None:
            print("You: " + audio_prompt)
            response = generate_response(audio_prompt)
            engine.say(response)
            engine.runAndWait()

            print("Robot: " + response)
            # to lower case
            if audio_prompt.lower() == "thank you":
                # Exit the program
                exit()
        else:
            print("Can you repeat?")
            continue


@safe_disconnect
def evaluate(
    robot: Robot, 
    chain_path: str | None = None,
    fps: int | None = None, 
    teleop_time_s: float | None = None, 
    display_cameras: bool = True
):
    robot_cfg = init_hydra_config(chain_path)

    models = robot_cfg["models"]
    policies = []
    for model_name in models:
        model = models[model_name]
        policy_overrides = ["device=cpu"]
        policy, policy_fps, device, use_amp = init_policy(model["repo_id"], policy_overrides)
        policies.append({"policy": policy, "policy_fps": policy_fps, "device": device, "use_amp": use_amp, "control_time_s": model["control_time_s"]})

    listener, events = init_keyboard_listener()
    for policy_obj in policies:
        control_loop(
            robot=robot,
            control_time_s=policy_obj["control_time_s"],
            display_cameras=display_cameras,
            events=events,
            policy=policy_obj["policy"],
            device=policy_obj["device"],
            use_amp=policy_obj["use_amp"],
            fps = policy_obj["policy_fps"],
            teleoperate=False,
        )
        print("Model is done!")

    print("Teleoperation is done!")


@safe_disconnect
def record(
    robot: Robot,
    root: str,
    repo_id: str,
    pretrained_policy_name_or_path: str | None = None,
    policy_overrides: List[str] | None = None,
    fps: int | None = None,
    warmup_time_s=2,
    episode_time_s=10,
    reset_time_s=5,
    num_episodes=50,
    video=True,
    run_compute_stats=True,
    push_to_hub=True,
    tags=None,
    num_image_writer_processes=1,
    num_image_writer_threads_per_camera=1,
    force_override=False,
    display_cameras=False,
    play_sounds=True,
):
    # TODO(rcadene): Add option to record logs
    listener = None
    events = None
    policy = None
    device = None
    use_amp = None

    # Load pretrained policy
    if pretrained_policy_name_or_path is not None:
        policy, policy_fps, device, use_amp = init_policy(pretrained_policy_name_or_path, policy_overrides)

        if fps is None:
            fps = policy_fps
            logging.warning(f"No fps provided, so using the fps from policy config ({policy_fps}).")
        elif fps != policy_fps:
            logging.warning(
                f"There is a mismatch between the provided fps ({fps}) and the one from policy config ({policy_fps})."
            )


    # Create empty dataset or load existing saved episodes
    sanity_check_dataset_name(repo_id, policy)
    dataset = init_dataset(
        repo_id,
        root,
        force_override,
        fps,
        video,
        write_images=robot.has_camera,
        num_image_writer_processes=num_image_writer_processes,
        num_image_writer_threads=num_image_writer_threads_per_camera * robot.num_cameras,
    )

    if not robot.is_connected:
        robot.connect()

    listener, events = init_keyboard_listener()

    # Execute a few seconds without recording to:
    # 1. teleoperate the robot to move it in starting position if no policy provided,
    # 2. give times to the robot devices to connect and start synchronizing,
    # 3. place the cameras windows on screen
    enable_teleoperation = policy is None
    log_say("Warmup record", play_sounds)
    warmup_record(robot, events, enable_teleoperation, warmup_time_s, display_cameras, fps)

    if has_method(robot, "teleop_safety_stop"):
        robot.teleop_safety_stop()

    while True:
        if dataset["num_episodes"] >= num_episodes:
            break

        episode_index = dataset["num_episodes"]

        # Visual sign in terminal that a recording is starting
        print("============================================")
        print("============================================")
        print("===========  START RECORDING  ==============")
        print("============================================")
        print("============================================")
        print("============================================")

        log_say(f"Recording episode {episode_index}", play_sounds)
        record_episode(
            dataset=dataset,
            robot=robot,
            events=events,
            episode_time_s=episode_time_s,
            display_cameras=display_cameras,
            policy=policy,
            device=device,
            use_amp=use_amp,
            fps=fps,
        )

        # Execute a few seconds without recording to give time to manually reset the environment
        # Current code logic doesn't allow to teleoperate during this time.
        # TODO(rcadene): add an option to enable teleoperation during reset
        # Skip reset for the last episode to be recorded
        if not events["stop_recording"] and (
            (episode_index < num_episodes - 1) or events["rerecord_episode"]
        ):
            log_say("Reset the environment", play_sounds)
            reset_environment(robot, events, reset_time_s)
            # log_say("Prepare position", play_sounds)
            # warmup_record(robot, events, enable_teleoperation, warmup_time_s, display_cameras, fps)

        if events["rerecord_episode"]:
            log_say("Re-record episode", play_sounds)
            events["rerecord_episode"] = False
            events["exit_early"] = False
            delete_current_episode(dataset)
            continue

        # Increment by one dataset["current_episode_index"]
        save_current_episode(dataset)

        if events["stop_recording"]:
            break

    log_say("Stop recording", play_sounds, blocking=True)
    stop_recording(robot, listener, display_cameras)

    lerobot_dataset = create_lerobot_dataset(dataset, run_compute_stats, push_to_hub, tags, play_sounds)

    data_dict = ["observation.images.laptop", "observation.images.phone"]
    image_keys = [key for key in data_dict if "image" in key]
    local_dir = Path(root) / repo_id
    videos_dir = local_dir / "videos"

    for episode_index in tqdm.tqdm(range(num_episodes)):
        for key in image_keys:
            # key = f"observation.images.{name}"
            tmp_imgs_dir = videos_dir / f"{key}_episode_{episode_index:06d}"
            shutil.rmtree(tmp_imgs_dir, ignore_errors=True)

    log_say("Exiting", play_sounds)
    return lerobot_dataset


@safe_disconnect
def replay(
    robot: Robot, episode: int, fps: int | None = None, root="data", repo_id="lerobot/debug", play_sounds=True
):
    # TODO(rcadene, aliberts): refactor with control_loop, once `dataset` is an instance of LeRobotDataset
    # TODO(rcadene): Add option to record logs
    local_dir = Path(root) / repo_id
    if not local_dir.exists():
        raise ValueError(local_dir)

    dataset = LeRobotDataset(repo_id, root=root)
    items = dataset.hf_dataset.select_columns("action")
    from_idx = dataset.episode_data_index["from"][episode].item()
    to_idx = dataset.episode_data_index["to"][episode].item()

    if not robot.is_connected:
        robot.connect()

    log_say("Replaying episode", play_sounds, blocking=True)
    for idx in range(from_idx, to_idx):
        start_episode_t = time.perf_counter()

        action = items[idx]["action"]
        robot.send_action(action)

        dt_s = time.perf_counter() - start_episode_t
        busy_wait(1 / fps - dt_s)

        dt_s = time.perf_counter() - start_episode_t
        log_control_info(robot, dt_s, fps=fps)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="mode", required=True)

    # Set common options for all the subparsers
    base_parser = argparse.ArgumentParser(add_help=False)
    base_parser.add_argument(
        "--robot-path",
        type=str,
        default="lerobot/configs/robot/koch.yaml",
        help="Path to robot yaml file used to instantiate the robot using `make_robot` factory function.",
    )
    base_parser.add_argument(
        "--robot-overrides",
        type=str,
        nargs="*",
        help="Any key=value arguments to override config values (use dots for.nested=overrides)",
    )

    parser_calib = subparsers.add_parser("calibrate", parents=[base_parser])
    parser_calib.add_argument(
        "--arms",
        type=str,
        nargs="*",
        help="List of arms to calibrate (e.g. `--arms left_follower right_follower left_leader`)",
    )

    parser_teleop = subparsers.add_parser("teleoperate", parents=[base_parser])
    parser_teleop.add_argument(
        "--fps", type=none_or_int, default=None, help="Frames per second (set to None to disable)"
    )
    parser_teleop.add_argument(
        "--display-cameras",
        type=int,
        default=1,
        help="Display all cameras on screen (set to 1 to display or 0).",
    )


    parser_evaluate = subparsers.add_parser("evaluate", parents=[base_parser])
    parser_evaluate.add_argument(
        "--fps", type=none_or_int, default=None, help="Frames per second (set to None to disable)"
    )
    parser_evaluate.add_argument(
        "--chain-path",
        type=Path,
        default="core/configs/chains/lamp_testing.yaml",
        help="Path to chain configuration yaml file').",
    )    


    parser_llm_agent = subparsers.add_parser("llm_agent", parents=[base_parser])
    parser_llm_agent.add_argument(
        "--chain-path",
        type=Path,
        default="core/configs/chains/clean_whiteboard.yaml",
        help="Path to chain configuration yaml file').",
    )    

    parser_record = subparsers.add_parser("record", parents=[base_parser])
    parser_record.add_argument(
        "--fps", type=none_or_int, default=None, help="Frames per second (set to None to disable)"
    )
    parser_record.add_argument(
        "--root",
        type=Path,
        default="data",
        help="Root directory where the dataset will be stored locally at '{root}/{repo_id}' (e.g. 'data/hf_username/dataset_name').",
    )
    parser_record.add_argument(
        "--repo-id",
        type=str,
        default="lerobot/test",
        help="Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).",
    )
    parser_record.add_argument(
        "--warmup-time-s",
        type=int,
        default=10,
        help="Number of seconds before starting data collection. It allows the robot devices to warmup and synchronize.",
    )
    parser_record.add_argument(
        "--episode-time-s",
        type=int,
        default=60,
        help="Number of seconds for data recording for each episode.",
    )
    parser_record.add_argument(
        "--reset-time-s",
        type=int,
        default=60,
        help="Number of seconds for resetting the environment after each episode.",
    )
    parser_record.add_argument("--num-episodes", type=int, default=50, help="Number of episodes to record.")
    parser_record.add_argument(
        "--run-compute-stats",
        type=int,
        default=1,
        help="By default, run the computation of the data statistics at the end of data collection. Compute intensive and not required to just replay an episode.",
    )
    parser_record.add_argument(
        "--push-to-hub",
        type=int,
        default=1,
        help="Upload dataset to Hugging Face hub.",
    )
    parser_record.add_argument(
        "--tags",
        type=str,
        nargs="*",
        help="Add tags to your dataset on the hub.",
    )
    parser_record.add_argument(
        "--num-image-writer-processes",
        type=int,
        default=0,
        help=(
            "Number of subprocesses handling the saving of frames as PNGs. Set to 0 to use threads only; "
            "set to ≥1 to use subprocesses, each using threads to write images. The best number of processes "
            "and threads depends on your system. We recommend 4 threads per camera with 0 processes. "
            "If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses."
        ),
    )
    parser_record.add_argument(
        "--num-image-writer-threads-per-camera",
        type=int,
        default=8,
        help=(
            "Number of threads writing the frames as png images on disk, per camera. "
            "Too many threads might cause unstable teleoperation fps due to main thread being blocked. "
            "Not enough threads might cause low camera fps."
        ),
    )
    parser_record.add_argument(
        "--force-override",
        type=int,
        default=0,
        help="By default, data recording is resumed. When set to 1, delete the local directory and start data recording from scratch.",
    )
    parser_record.add_argument(
        "-p",
        "--pretrained-policy-name-or-path",
        type=str,
        help=(
            "Either the repo ID of a model hosted on the Hub or a path to a directory containing weights "
            "saved using `Policy.save_pretrained`."
        ),
    )
    parser_record.add_argument(
        "--policy-overrides",
        type=str,
        nargs="*",
        help="Any key=value arguments to override config values (use dots for.nested=overrides)",
    )

    parser_replay = subparsers.add_parser("replay", parents=[base_parser])
    parser_replay.add_argument(
        "--fps", type=none_or_int, default=None, help="Frames per second (set to None to disable)"
    )
    parser_replay.add_argument(
        "--root",
        type=Path,
        default="data",
        help="Root directory where the dataset will be stored locally at '{root}/{repo_id}' (e.g. 'data/hf_username/dataset_name').",
    )
    parser_replay.add_argument(
        "--repo-id",
        type=str,
        default="lerobot/test",
        help="Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).",
    )
    parser_replay.add_argument("--episode", type=int, default=0, help="Index of the episode to replay.")
    
    parser_llm_agent.add_argument(
        "--typing",
        action="store_true",
        default=False,
        help="Enable chat-based input instead of voice commands.",
    )

    args = parser.parse_args()

    init_logging()

    control_mode = args.mode
    robot_path = args.robot_path
    robot_overrides = args.robot_overrides
    kwargs = vars(args)
    del kwargs["mode"]
    del kwargs["robot_path"]
    del kwargs["robot_overrides"]

    robot_cfg = init_hydra_config(robot_path, robot_overrides)
    robot = make_robot(robot_cfg)
    # robot.connect()
    # rest_position = robot.follower_arms['main'].read("Present_Position")
    rest_position = [  0.9667969 ,128.84766 ,  174.99023,   -16.611328,   -4.8339844  ,34.716797 ]
    # robot.disconnect()

    if control_mode == "calibrate":
        calibrate(robot, **kwargs)

    elif control_mode == "teleoperate":
        teleoperate(robot, **kwargs)

    elif control_mode == "evaluate":
        evaluate(robot, **kwargs)

    elif control_mode == "llm_agent":
        llm_agent(robot, **kwargs)

    elif control_mode == "record":
        record(robot, **kwargs)

    elif control_mode == "replay":
        replay(robot, **kwargs)

    if robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
        robot.disconnect()