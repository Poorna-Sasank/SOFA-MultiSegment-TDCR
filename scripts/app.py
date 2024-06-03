import tkinter
import threading
import Sofa

class App(threading.Thread):
    
    '''
        : Make the App dynamically adjust to the number of cables
        : Try to establish communication with Arduino port or internet sockets for data transfer
    '''
    nbCables = 12
    
    # Define cable names
    cable_names = [
        "Cable 1",
        "Cable 2",
        "Cable 3",
        "Cable 4",
        "Cable 5",
        "Cable 6",
        "Cable 7",
        "Cable 8",
        "Cable 9",
        "Cable 10",
        "Cable 11",
        "Cable 12"
    ]

    def __init__(self, controller, init_values=[0.0] * nbCables):
        threading.Thread.__init__(self)
        self.daemon = True
        self.controller = controller
        self.init_values = init_values
        self.start()
        
        self.cablesInit = [0.0] * self.nbCables
        # for i in range(self.nbCables):
        #     self.cablesInit[i] = init_values[i]
        
    def callback(self):
        self.root.quit()
        
    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)
        self.root.title("Robot Controller Interface")

        self.cables = []
        self.labels = []
        
        for i in range(self.nbCables):
            # A DoubleVar for each cable
            cable_var = tkinter.DoubleVar(value=self.init_values[i])
            self.cables.append(cable_var)
            
            # A label for the cable name
            label = tkinter.Label(self.root, text=self.cable_names[i])
            label.grid(row=0, column=i)
            self.labels.append(label)

            # A Scale widget for the cable displacement
            scale = tkinter.Scale(self.root, variable=cable_var, resolution=1, length=200, from_=0, to=5, orient=tkinter.VERTICAL)
            scale.grid(row=1, column=i)

        self.root.mainloop()


class CableController(Sofa.Core.Controller):
    nbCables = 12
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cables = args
        self.name = "FingerController"

    def onAnimateBeginEvent(self, event):
        cables = [0.0] * self.nbCables    
        for i, cable in enumerate(self.cables):
            cable.CableConstraint.value = [self.app.cables[i].get()]
        
        return cables
