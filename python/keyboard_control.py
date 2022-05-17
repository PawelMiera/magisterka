import keyboard
import threading

class Keyboard_Listener:
    def __init__(self):
        self.v_x = 0
        self.v_y = 0
        self.v_z = 0
        self.v_yaw = 0
        self.speed = 0.5
        x = threading.Thread(target=self.keyboard_listener)
        x.start()

        print("running")

    def take_off(self):
        pass

    def land(self):
        pass

    def rtl(self):
        pass

    def increase_speed(self, e):
        self.speed += 0.2

    def decrease_speed(self, e):
        if self.speed > 0.2:
            self.speed -= 0.2

    def keyboard_listener(self):
        keyboard.on_release_key("m", self.increase_speed)
        keyboard.on_release_key("n", self.decrease_speed)

        while True:
            if keyboard.is_pressed("esc"):
                break
            if keyboard.is_pressed("t"):
                self.take_off()
            if keyboard.is_pressed("l"):
                self.land()
            if keyboard.is_pressed("r"):
                self.rtl()
            if keyboard.is_pressed("w"):
                self.v_x = self.speed
            elif keyboard.is_pressed("s"):
                self.v_x = - self.speed
            else:
                self.v_x = 0

            if keyboard.is_pressed("a"):
                self.v_y = self.speed
            elif keyboard.is_pressed("d"):
                self.v_y = - self.speed
            else:
                self.v_y = 0

            if keyboard.is_pressed("z"):
                self.v_z = self.speed
            elif keyboard.is_pressed("x"):
                self.v_z = - self.speed
            else:
                self.v_z = 0

            if keyboard.is_pressed("q"):
                self.v_yaw = self.speed
            elif keyboard.is_pressed("e"):
                self.v_yaw = - self.speed
            else:
                self.v_yaw = 0

            print(self.v_x, self.v_y, self.v_z, self.v_yaw, self.speed)


keyboard_control = Keyboard_Listener()