import pandas as pd


class ambf_raven_recorder:

    def __init__(self):
        self.rs_df = None
        self.ci_df = None
        self.writer = None
        self.file = None

    def build_rs_headers(self):
        """
        Creates an array of stings to be used as headers for raven state recording
        """

        headers = ["time"]

        # Headers for jpos
        for i in range(16):
            headers.append("jpos" + str(i))  # 7 numbers

        # Headers for runlevel, sublevel, and last_seq
        headers.extend(["runlevel", "sublevel", "last_seq"])

        # Headers for type
        for i in range(2):
            headers.append("type" + str(i))

        # Headers for pos
        for i in range(6):
            headers.append("pos" + str(i))

        # Headers for ori
        for i in range(18):
            headers.append("ori" + str(i))

        # Headers for ori_d
        for i in range(18):
            headers.append("ori_d" + str(i))

        # Headers for pos_d
        for i in range(6):
            headers.append("pos_d" + str(i))

        # Headers for encVals
        for i in range(16):
            headers.append("encVals" + str(i))

        # Headers for dac_val
        for i in range(16):
            headers.append("dac_vals" + str(i))

        # Headers for Tau
        for i in range(16):
            headers.append("Tau" + str(i))

        # Headers for mpos
        for i in range(16):
            headers.append("mpos" + str(i))

        # Headers for mvel
        for i in range(16):
            headers.append("mvel" + str(i))

        # Headers for jvel for both arms
        for i in range(16):
            headers.append("jvel" + str(i))

        # Headers for jpos_d
        for i in range(16):
            headers.append("jpos_d" + str(i))

        # Headers for grasp_d
        for i in range(16):
            headers.append("grasp_d" + str(i))

        # Headers for encoffsets
        for i in range(16):
            headers.append("encoffsets" + str(i))

        # Headers for jac_vel
        for i in range(12):
            headers.append("jac_vel" + str(i))

        # Placeholders for jac_f
        for i in range(12):
            headers.append("jac_f" + str(i))

        return headers

    def record_raven_status(self):
        """
        Initializes the dataframe in a format matching the raven status rosbag recorder
        """
        self.rs_df = pd.DataFrame(columns=self.build_rs_headers())
        print("Now recording raven status")

    def write_raven_status(self, raven):
        """
        Writes the current raven status to the dataframe
        """
        newline = raven.get_raven_status()
        self.rs_df.loc[len(self.rs_df.index)] = newline

    def build_ci_headers(self):
        """
        Creates an array of strings to be used as headers for controller inputs recording
        """

        headers = ["Left Joystick X", "Left Joystick Y", "Left Trigger", "Left Bumper",
                   "Right Joystick X", "Right Joystick Y", "Right Trigger", "Right Bumper",
                   "A Button", "B Button", "X Button", "Y Button", "Back Button", "Start Button"]

        return headers

    def record_controller_inputs(self):
        """
        Initializes the dataframe in a format for recording controller inputs
        """
        self.ci_df = pd.DataFrame(columns=self.build_ci_headers())
        print("Now recording controller inputs")

    def write_controller_inputs(self, controller_inputs):
        """
        Writes the current controller inputs to the dataframe
        """
        formatted_controller_inputs = []

        for i in range(len(controller_inputs)):
            formatted_controller_inputs.extend(controller_inputs[i])

        self.ci_df.loc[len(self.ci_df.index)] = formatted_controller_inputs

    def stop_recording(self, filename="test"):
        """
        Stops the recording by writing out the current dataframe a CSV to the specified file path
        Args:
            filename : the path to/filename of the csv to be saved
        """
        filename_rs = filename + "_rs.csv"
        filename_ci = filename + "_ci.csv"

        if self.rs_df is not None:
            self.rs_df.to_csv(filename_rs, encoding='utf-8', index=False)
            self.rs_df = None
        if self.ci_df is not None:
            self.ci_df.to_csv(filename_ci, encoding='utf-8', index=False)
            self.ci_df = None
