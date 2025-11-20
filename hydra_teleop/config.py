# config.py
from dataclasses import dataclass
import os
from typing import Optional, Tuple, List

@dataclass
class TeleopConfig:
    # --- Run Options ---
    simulation_runs = 500
    simulation_timeout = 200
    simulation_world_style = "city" #"perlin|city"
    # --- Sim / world ---
    world_path: str = "/home/vikhyat-chauhan/Documents/Hydra/worlds/airport_world.sdf"
    #sim_cmd: tuple[str, ...] = ("gz", "sim", "-r")  # GUI
    sim_cmd: tuple[str, ...] = ("gz", "sim", "4", "-s", "-r")  # headless
    sim_env: dict | None = None
    sim_boot_secs: float = 8.0
    world_gen_seed_offset = 112
    target_distance = 150
    perlin_seed_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 150, 151, 152, 153, 154, 155, 156, 157, 158, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 208, 209, 210, 211, 212, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554, 555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575, 576, 577, 578, 579, 580, 581, 582, 583, 584, 585, 586, 587, 588, 589, 590, 591, 593, 594, 595, 596, 597, 598, 599, 600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 611, 612, 613, 614, 615, 616, 617, 618, 619, 620, 621, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637, 638, 639, 640, 641, 642, 643, 644, 645, 646, 647, 648, 649, 650, 651, 652, 653, 654, 655, 656, 658, 659, 660, 661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 676, 677, 678, 679, 680, 681, 682, 683, 684, 685, 686, 687, 688, 689, 690, 691, 692, 693, 694, 695, 697, 698, 699, 700, 701, 702, 703, 704, 705, 706, 707, 708, 709, 710, 711, 712, 713, 714, 715, 716, 717, 718, 719, 720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 733, 734, 735, 736, 737, 738, 739, 740, 741, 742, 743, 744, 745, 746, 747, 748, 749, 750, 751, 752, 753, 754, 755, 756, 757, 758, 759, 760, 761, 762, 763, 764, 765, 766, 767, 768, 769, 771, 772, 773, 774, 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824, 825, 826, 827, 828, 829, 830, 831, 832, 833, 834, 835, 837, 838, 839, 840, 841, 842, 844, 845, 846, 847, 848, 849, 850, 851, 852, 853, 854, 855, 856, 857, 858, 859, 860, 861, 862, 863, 864, 865, 867, 868, 869, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879, 880, 881, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891, 892, 893, 895, 896, 897, 898, 899, 901, 902, 903, 904, 905, 906, 907, 908, 909, 910, 911, 912, 913, 914, 915, 916, 917, 918, 919, 920, 921, 922, 923, 924, 926, 927, 928, 929, 930, 931, 932, 933, 934, 935, 936, 937, 938, 940, 941, 942, 943, 944, 945, 946, 947, 948, 950, 951, 952, 953, 954, 955, 956, 958, 959, 960, 962, 963, 964, 965, 966, 967, 968, 969, 970, 971, 972, 973, 974, 975, 976, 977, 978, 979, 981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 994, 995, 996, 997, 998, 999] \
    #[3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 25, 26, 27, 28, 30, 31, 33, 34, 35, 37, 38, 39, 42, 43, 44, 45, 52, 54, 55, 56, 57, 58, 59, 60, 61, 63, 64, 65, 68, 69, 70, 71, 72, 73, 74, 75, 79, 80, 81, 82, 87, 88, 89, 91, 95, 96, 97, 98, 100, 102, 103, 105, 106, 108, 109, 110, 111, 112, 113, 115, 116, 117, 118, 119, 122, 124, 125, 126, 128, 129, 130, 131, 132, 133, 135, 136, 137, 139, 141, 143, 144, 145, 146, 147, 148, 149, 154, 155, 156, 157, 158, 160, 162, 163, 164, 165, 166, 167, 172, 173]
    # --- Gazebo transport ---
    topic: str = "/model/drone1/cmd_vel"  # gz.msgs.Twist
    world_pose_topic: str = "/world/airport/pose/info"

    # --- Teleop dynamics ---
    rate_hz: float = 30
    speed_x: float = 5.0
    speed_y: float = 5.0
    speed_z: float = 5.0
    yaw_rate: float = 0.8

    # --- Logging ---
    log_path: str = "logs/run_logs.json"

    def __post_init__(self):
        if self.sim_env is None:
            self.sim_env = {
                **os.environ,
                "QT_QPA_PLATFORM": "xcb",
                "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
                "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            }

    # =======================
    # RESULTS & ANALYZER CFG
    # =======================

    # Where main experiment writes the consolidated results CSV:
    results_csv_path: str = "logs/results/experiment_summary.csv"

    # Analyzer output directory (plots + summaries will go here):
    analyzer_out_dir: str = "logs/results"

    # Optional explicit strategy names (exactly two); leave None to auto-detect
    analyzer_pair_order: Optional[Tuple[str, str]] = None
    # e.g. analyzer_pair_order = ("NAVALGO1", "NAVALGO2")

    # Plot toggles
    analyzer_annotate_points: bool = False
    # If True, keep runs where either strategy timed out; if False, drop them
    analyzer_include_timeouts: bool = True
    analyzer_strategies = ["APE1", "APE2", "APE3", "TROOP"]
    #analyzer_strategies = ["APE2"]

    # =======================
    # Generated Simulation & Algo Selector
    # =======================
    targets_json_path = "models/generated/targets.json"  # your file

    time_budget_s = 60                                   # optional
    n_repeats = 5
    timeout_s = 500

    # Fixed start pose for selector (no ROS needed for this)
    start_x: float = -95.0
    start_y: float =   0.0
    start_z: float =   1.0

    # (optional, for completeness if any code uses orientation)
    start_roll:  float = 0.0
    start_pitch: float = 0.0
    start_yaw:   float = 0.0  # radians
    # Target file path (from your earlier JSON)
    target_json_path: str = "models/generated/generated_target_meta.json"

    # Optional: experiment knobs
    n_repeats: int = 1
    timeout_s: int = 500
    time_budget_s: float | None = None  # set a number if you want a budget

    # =======================
    # Event Generator (B-style)
    # =======================
    # Topics
    event_topic = "/hydra/event"
    ros_pose_topic = "/model/drone1/pose/info"

    # -----------------------
    # Time constraint model
    # -----------------------
    # Event spacing follows a log-uniform distribution between these bounds.
    # The time between two events (Δt) is the *actual constraint*.
    # Short intervals create overlapping / near-deadline pressure on APEs.
    event_dt_min_s = 0.08   # lower bound (tightest events)
    event_dt_max_s = 3.5    # upper bound (slow cycles)

    # Optional single global deadline for legacy consumers.
    # Usually left None (since Δt itself acts as the time constraint).
    event_global_deadline_s = None

    # -----------------------
    # Mixture of event types
    # -----------------------
    # Defines probabilities for each kind.
    event_mix_enemy = 0.33
    event_mix_obstacle = 0.33
    event_mix_lane = 0.34

    # -----------------------
    # Arena bounds
    # -----------------------
    event_bounds_xy = (-100.0, 100.0, -50.0, 50.0)

    # -----------------------
    # Logging
    # -----------------------
    event_log_csv_path = "logs/events_log.csv"
    
    deadline_alpha: float = 0.85
    deadline_min_s: float = 0.12
    deadline_max_s: float = 1.20
    # -----------------------
    # Physics
    # -----------------------
    # Deterministic
    cmd_latency_s: float = 0.0 #0.10
    wind_level_0to1: float = 0.0 #0.5
    wind_accel_std_base_mps2: float = 0.0 #0.8
    '''
    # Non Deterministic
    cmd_latency_s: float = 0.10
    wind_level_0to1: float = 0.5
    wind_accel_std_base_mps2: float = 0.8  '''
