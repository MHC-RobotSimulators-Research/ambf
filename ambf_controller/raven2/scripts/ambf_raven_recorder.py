import csv


class file_recorder:

    def __init__(self, raven):
        self.raven = raven
        self.writer = None

    def start_recording(self, filename):
        with open(filename, 'wb') as file:
            self.writer = csv.writer()
            print("Now recording to: ", filename)

    def write_line(self):
        line = raven.get_raven_status(0, True)
        self.writer.writerow(line)

    def stop_recording(self):
        self.writer = None
