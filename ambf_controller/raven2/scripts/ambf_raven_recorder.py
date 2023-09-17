import csv
import pandas as pd


class ambf_raven_recorder:

    def __init__(self):
        self.df = None
        self.writer = None
        self.file = None

    def build_rs_headers(self):
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
        self.df = pd.DataFrame(columns=self.build_rs_headers())
        print("Now recording raven status")

    def write_raven_status(self, raven):
        newline = raven.get_raven_status()
        self.df.loc[len(self.df.index)] = newline

    def stop_recording(self, filename="test.csv"):
        self.df.to_csv(filename, encoding='utf-8')
