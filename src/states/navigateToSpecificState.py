from states.navigate import NavigateState

'''Child Classes to Process Navigation around each Room'''
class NavigateToAState(NavigateState):
    def __init__(self):
        super().__init__()
        self.duration = 30.00
        self.min_x = 0.5
        self.max_x = 3
        self.min_y = 6
        self.max_y = 10

    def set_next_room(self, ud):
        ud.next_room = 'B'


class NavigateToBState(NavigateState):
    def __init__(self):
        super().__init__()
        self.duration = 30.00
        self.min_x = 4
        self.max_x = 8
        self.min_y = 7
        self.max_y = 10

    def set_next_room(self, ud):
        ud.next_room = 'D'


class NavigateToDState(NavigateState):
    def __init__(self):
        super().__init__()
        self.duration = 10.00
        self.min_x = 0.5
        self.max_x = 3
        self.min_y = 0.5
        self.max_y = 5

    def set_next_room(self, ud):
        ud.next_room = 'A'




