#!/usr/bin/env python
import math
import numpy

step = 0.05

# start_pose = [0.5, 0.3, 0.5] #given
# end_pose = [1, 1, 0.5]     #given


# output_marker = []
# StepCount = 0

# def CompilePose(inter):
#    print(inter)
#    output_marker = []
#    placeholder = [0] * 3
#    while(len(inter) != 0):
#       i = 0
#       for i in range(3):
#          placeholder[i] = inter.pop(0)
#       print(placeholder)
#       output_marker.append(placeholder)
#    print(output_marker)
#    return output_marker
   


def FindStepsNo(start, end):
   # print(start)
   # print(end)
   i = 0
   steplist = [0] * 3
   for i in range(3):
      steplist[i] = math.ceil(abs((end[i] - start[i]))/step)
   # print(steplist)
   
   return int(max(steplist))

def GeneratePose(calculatedstep, start, end):
   inter_marker = []
   increament = [0] * 3
   i = 0
   for i in range(3):
      increament[i] = round((end[i] - start[i])/calculatedstep, 3)
   # print(increament)
   # print("=======")
   # print("start: " + str(start))
   # print("end: " + str(end))
   # print("=======")
   gen_pose = [0, 0, 0]
   for i in range(calculatedstep-1):
      j = 0
      for j in range(3):
         gen_pose[j] = start[j] + (i+1)*increament[j]
      # print(gen_pose)
      inter_marker.extend(gen_pose)
   fake_end = [x + 0.00001 for x in end]
   inter_marker.extend(fake_end)
   # print(fake_end)
   # print(inter_marker[len(inter_marker)-1])
   # print(len(inter_marker))
   return inter_marker

def main(start_pose, end_pose):
   StepCount = FindStepsNo(start_pose, end_pose)
   return(GeneratePose(StepCount, start_pose, end_pose), StepCount)

# if __name__ == "__main__":
   # retractor1_processed_position = [-0.3, 0.365, 0.4]
   # retractor2_processed_position = [-0.35, 0.365, 0.4]

   # retractor_size = [0.03, 0.2, 0.03]

   # wbo_ret1 = [retractor1_processed_position[0], retractor1_processed_position[1], retractor1_processed_position[2] + 0.5*retractor_size[2], -0.5, 0.5, 0.5, 0.5]
   # wbo_ret2 = [retractor2_processed_position[0], retractor2_processed_position[1], retractor2_processed_position[2] + 0.5*retractor_size[2], -0.5, 0.5, 0.5, 0.5]
   # start_pos = [-0.331084150613, 0.298791543686, 1.15772468869]
   
   # end_pos = [wbo_ret2[0], wbo_ret2[1], wbo_ret2[2]]

   # start_pos = [0.0,0.0,1.0]
   # end_pos = [0.0,0.0,1.705025]
   # market, step = main(start_pos, end_pos)