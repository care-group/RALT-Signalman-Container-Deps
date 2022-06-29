import os

class FolderCheckCreate():
    def run(self, participant):
        path = '/home/sandbox/output/' + 'P' + str(participant)
        exists = self.check_for_folder(path)
        if not exists:
            self.create_folder(path)

    def check_for_folder(self, path):
        exists = os.path.isdir(path)

        return exists

    def create_folder(self, path):
        try:
            os.mkdir(path)
        except OSError:
            msg = 'Unable to create new directory at path:' + path + '. Folder may already exist.'
            print(msg)
        else:
            msg = 'Successfully created new directory for participant at path: ' + path
            print(msg)