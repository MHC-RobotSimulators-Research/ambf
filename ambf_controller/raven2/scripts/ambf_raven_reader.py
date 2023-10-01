import pandas as pd


class ambf_raven_reader:

    def __init__(self):
        self.df = None
        self.status = False
        self.row = 0
        self.length = 0

    def get_status(self):
        return self.status

    def load_csv(self, filename, type_of_csv):
        self.df = pd.read_csv(filename)
        if type_of_csv == "controller":
            if self.df.shape[1] == 14:
                self.status = True
                self.length = self.df.shape[0]
                print("Successfully loaded ", filename)
                return True
            else:
                return False
        elif type_of_csv == "jpos":
            if self.df.shape[1] == 240:
                self.status = True
                self.length = self.df.shape[0]
                print("Successfully loaded ", filename)
                return True
            else:
                return False

    def read_ci(self):
        if self.row < self.length:
            row = self.df.loc[self.row, :].values.flatten().tolist()
            controller = [row[0:4], row[4:8], row[8:14]]
            self.row += 1
            return controller

        else:
            self.df = None
            self.status = False
            self.row = 0
            self.length = 0
