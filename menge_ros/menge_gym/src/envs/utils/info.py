class Timeout(object):
    def __init__(self):
        pass

    def __str__(self):
        return 'Timeout'


class ReachGoal(object):
    def __init__(self):
        pass

    def __str__(self):
        return 'Reaching goal'


class Discomfort(object):
    def __init__(self, min_dist):
        self.min_dist = min_dist

    def __str__(self):
        return 'Discomfort'


class Clearance(object):
    def __init__(self, min_dist):
        self.min_dist = min_dist

    def __str__(self):
        return 'Clearance'


class Collision(object):
    def __init__(self, target=None):
        self.target = target

    def __str__(self):
        ret_str = 'Collision'
        if self.target:
            ret_str += ' with ' + self.target
        return ret_str


class Nothing(object):
    def __init__(self):
        pass

    def __str__(self):
        return ''
