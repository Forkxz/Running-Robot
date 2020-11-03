import numpy as np

################################################颜色区域函数#################################################
DIM=(640, 480)
color_range = {
               'yellow_door': [(20, 140, 60), (40, 240, 150)],
               'black_door': [(25, 25, 10), (110, 150, 30)],
               'black_door_new': [(0, 0, 0), (255, 255, 77)],
               'black_gap': [(0, 0, 0), (180, 255, 70)],
               'yellow_hole': [(20, 120, 95), (30, 250, 190)],
               'black_hole': [(5, 80, 20), (40, 255, 100)],
               'chest_red_floor': [(0, 40, 60), (20,200, 190)],
               'chest_red_floor1': [(0, 100, 60), (20,200, 210)],
               'chest_red_floor2': [(110, 100, 60), (180,200, 210)],
                'green_bridge': [(50, 75, 70), (80, 240, 210)],
               'red blue green':[(0,0,0), (255,145,255)]
               }

color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'black_dir': {'Lower': np.array([0, 0, 10]), 'Upper': np.array([170, 170, 45])},
              'black_line': {'Lower': np.array([0, 0, 20]), 'Upper': np.array([100, 160, 80])},
              'blue': {'Lower': np.array([87, 150, 50]), 'Upper': np.array([103, 240, 180])},
              #'blue': {'Lower': np.array([91, 74, 56]), 'Upper': np.array([115, 255, 171])},
              'ball_red': {'Lower': np.array([160, 100, 70]), 'Upper': np.array([190, 215, 145])},
              'blue_hole': {'Lower': np.array([100, 130, 80]), 'Upper': np.array([130, 255, 150])},
              'flap': {'Lower': np.array([52, 110, 47]), 'Upper': np.array([178, 216, 221])},
              'landmine': {'Lower': np.array([50, 30, 20]), 'Upper': np.array([110, 140, 70])},
              'step': {'Lower': np.array([53, 142, 53]), 'Upper': np.array([171, 255, 204])},
              }

color_dist_lza = {
    'flap': {'Lower': np.array([85,86,23]), 'Upper': np.array([121,242,180])}
}

#               'breaking_bar': [(84, 65, 63), (113, 225, 132)],
#               'blue_door': [(91, 74, 56), (115, 255, 171)],
#               'step': [(53, 142, 53), (171, 255, 204)],
#               'final': [(0, 137, 98), (39, 225, 226)]