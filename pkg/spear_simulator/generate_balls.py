# Generates n balls in an x by y region (centered at 0 0 0) at height z
import sys
from random import uniform, randint


def generate_balls(n, x_length, y_length, z_height, f_name):
    # Determine boundaries
    x_upper_bound = x_length / 2
    x_lower_bound = -x_upper_bound
    y_upper_bound = y_length / 2
    y_lower_bound = -y_upper_bound

    # Define possible ball file names:
    ball_names = [
        "tennis_fluorescent_yellow", "tennis_lime", "tennis_green",
        "tennis_yellow"
    ]

    # print to a document following the format in .world files
    for n in range(n):
        x_random = uniform(x_lower_bound, x_upper_bound)
        y_random = uniform(y_lower_bound, y_upper_bound)

        randball = ball_names[randint(0, len(ball_names) - 1)]
        count = str(n)
        ballname = randball + count
        if n == 0:
            f = open(f_name, "w+")
            f2 = open(f_name + "2", "w+")
            write_first(n, x_random, y_random, z_height, f, randball, ballname)
            write_second(n, x_random, y_random, z_height, f2, randball,
                         ballname)
            f.close()
            f2.close()
        else:
            f = open(f_name, "a+")
            f2 = open(f_name + "2", "a+")
            write_first(n, x_random, y_random, z_height, f, randball, ballname)
            write_second(n, x_random, y_random, z_height, f2, randball,
                         ballname)
            f.close()
            f2.close()


# writes the first section to f


def write_first(n, x_random, y_random, z_height, f, randball, ballname):
    f.write("\
<model name='%s'>\n\
    <pose frame=''>%d %d %d 0 0 0</pose>\n\
    <scale>1 1 1</scale>\n\
    <link name='ball'>\n\
        <pose frame=''>%d %d %d 0 0 0</pose>\n\
        <velocity>0 0 0 0 0 0</velocity>\n\
        <acceleration>0 0 0 0 0 0</acceleration>\n\
        <wrench>0 0 0 0 0 0</wrench>\n\
    </link>\n\
</model>\n" % (ballname, x_random, y_random, z_height, x_random, y_random,
               z_height))


# writes the second section to f2


def write_second(n, x_random, y_random, z_height, f2, randball, ballname):
    f2.write("\
<model name='%s'>\n\
  <static>0</static>\n\
  <link name='ball'>\n\
    <pose frame=''>0 0 1 0 -0 0</pose>\n\
    <collision name='collision'>\n\
      <geometry>\n\
        <mesh>\n\
          <uri>model://%s/%s.dae</uri>\n\
        </mesh>\n\
      </geometry>\n\
      <surface>\n\
        <bounce>\n\
          <restitution_coefficient>0.725</restitution_coefficient>\n\
          <threshold>0</threshold>\n\
        </bounce>\n\
        <contact>\n\
          <ode>\n\
            <max_vel>10</max_vel>\n\
          </ode>\n\
        </contact>\n\
        <friction>\n\
          <torsional>\n\
            <ode/>\n\
          </torsional>\n\
          <ode/>\n\
        </friction>\n\
      </surface>\n\
      <max_contacts>10</max_contacts>\n\
    </collision>\n\
    <visual name='visual'>\n\
      <geometry>\n\
        <mesh>\n\
          <uri>model://%s/%s.dae</uri>\n\
        </mesh>\n\
      </geometry>\n\
    </visual>\n\
    <inertial>\n\
      <mass>0.055</mass>\n\
      <inertia>\n\
        <ixx>5.19527</ixx>\n\
        <ixy>0</ixy>\n\
        <ixz>0</ixz>\n\
        <iyy>5.15681</iyy>\n\
        <iyz>0</iyz>\n\
        <izz>5.15681</izz>\n\
      </inertia>\n\
      <pose frame=''>0 0 0 0 -0 0</pose>\n\
    </inertial>\n\
    <self_collide>0</self_collide>\n\
    <enable_wind>0</enable_wind>\n\
    <kinematic>0</kinematic>\n\
  </link>\n\
  <pose frame=''>%d %d %d 0 0 0</pose>\n\
</model>\n" % (ballname, randball, randball, randball, randball, x_random,
               y_random, z_height))


if __name__ == "__main__":
    while True:
        userin = input(
            "Enter: #ofballs x_length y_length z_height f_name: ").split()
        if len(userin) != 5:
            print("Incorrect amount of arguments")
        else:
            break
    generate_balls(
        int(userin[0]), float(userin[1]), float(userin[2]), float(userin[3]),
        userin[4])
