from omni.kit.scripting import BehaviorScript
import math
import time
from pxr import Gf

# user orbit parameters
period = 10 # period in seconds
semimajor = 1270 # semimajor axis in world units
semiminor = 1190 # semiminor axis in world units
latitude = 70 # latitude of rover/center of terrain

# angle offset from where rover is
theta = (90 - latitude) * math.pi/180 
omega = -360 / period

class OrbiterPath(BehaviorScript):
    def on_init(self):
        print(f"{__class__.__name__}.on_init()->{self.prim_path}")

        self._prim = self.stage.GetPrimAtPath(self.prim_path)

    def on_destroy(self):
        print(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        print(f"{__class__.__name__}.on_play()->{self.prim_path}")

        self.start_time = time.time()

    def on_pause(self):
        print(f"{__class__.__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        print(f"{__class__.__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        print(f"{__class__.__name__}.on_update(current_time={current_time}, delta_time={delta_time})->{self.prim_path}")

        t = time.time() - self.start_time

        i = semimajor * math.cos((math.pi * t)/(period/2))
        j = semiminor * math.sin((math.pi * t)/(period/2))

        y = float(j * math.cos(theta))
        x = float(y * math.tan(theta))
        z = float(i)
        xyz = Gf.Vec3f(x,y,z)
        rotate = Gf.Vec3f(omega*t,0,-theta*180/math.pi)
        self._prim.GetAttribute("xformOp:translate").Set(xyz)
        self._prim.GetAttribute("xformOp:rotateXYZ").Set(rotate)