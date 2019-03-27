class UnsucessfulAllocationError(Exception):
    """ Raised when a task could not be allocated in the desired time slot.
    A suggested start time for the task is passed as an argument"""

    def __init__(self, suggested_start_time):
        Exception.__init__(self, suggested_start_time)
        self.suggested_start_time = suggested_start_time

#https://www.codementor.io/sheena/how-to-write-python-custom-exceptions-du107ufv9