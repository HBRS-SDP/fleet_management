class UnsuccessfulAllocationAlternativeTimeSlot(Exception):
    """ Raised when a task could not be allocated at the desired time slot.
    An alternative timeslot for the task is passed as an argument
     @param alternative_timeslots: dict (keys = task_ids, values = robot_id, start_time """

    def __init__(self, alternative_timeslots):
        Exception.__init__(self, alternative_timeslots)
        self.alternative_timeslots = alternative_timeslots
