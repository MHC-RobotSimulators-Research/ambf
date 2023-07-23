import csv


class ambf_raven_recorder:

    def __init__(self):
        self.writer = None
        self.file = None

    def start_recording(self, filename="test.csv"):
        self.file = open(filename, 'w')
        self.writer = csv.writer(self.file)
        print("Now recording to: ", filename)

    def write_raven_status(self, raven, first_entry=False):
        line = raven.get_raven_status(0, first_entry)
        # print(line)
        self.writer.writerow(line)

    def stop_recording(self):
        self.file.close()
        self.writer = None
