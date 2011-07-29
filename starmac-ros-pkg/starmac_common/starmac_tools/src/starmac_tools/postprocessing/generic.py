class Dummy(object):
    def __repr__(self):
        return self.__dict__.__repr__()

def get_header_time(bag, topic):
    return bag._data[topic+'/_header_time']
