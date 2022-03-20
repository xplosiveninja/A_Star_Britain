#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 20:47:44 2022

@author: Charlie Nicholls

A python script that will find the optimal route between 2 random points on the UK road network using 
the A_Star algorithm.

The road network is defined using an image in which the roads are isolated and have pixel values of 
[255, 255, 255, 255].
"""

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import imageio
import os

class Path_Finder:
    """
    Imports the road network from a png and converts to a Boolean Array
    """
    def Data_Import():
        #Load Image of UK with road network isolated and convert to numpy array
        Imported_Road_Map_Image = Image.open('UK_Roads_Map.png')
        Imported_Road_Map_Array = np.asarray(Imported_Road_Map_Image)
        
        #Create a Boolean Array and set all Locations with pixels values [255, 255, 255, 255] in the imported array to true
        Road_Map_Array = np.zeros((Imported_Road_Map_Array.shape[0:2]), dtype = np.bool)
        for y, Slice in enumerate(Imported_Road_Map_Array):
            for x, PixelValues in enumerate(Slice):
                if np.array_equal(PixelValues, np.array([255, 255, 255, 255])):
                    Road_Map_Array[-y, x] = 1
        return Road_Map_Array
    
    """
    Function to find 2 valid start and finish locations
    
    Map is a boolean map that is true for valid roads and false for invalid areas
    """
    def Random_Start_End(Map):
        #Find Non zero locations in map
        Valid_Locations = np.nonzero(Map)
        
        #Select 2 random set of indexes for the start and finishing locations. Find new finishing location if Start index = Finsih index
        Start_Selector = np.random.randint(0, Valid_Locations[0].shape)
        Finish_Selector = np.random.randint(0, Valid_Locations[0].shape)
        while Finish_Selector == Start_Selector:
            Finish_Selector = np.random.randint(0, Valid_Locations.shape[0])
        
        #Find Start, finish Indexes and flatten
        Start = np.array([Valid_Locations[0][Start_Selector], Valid_Locations[1][Start_Selector]])
        Start = Start.flatten()
        Finish = np.array([Valid_Locations[0][Finish_Selector], Valid_Locations[1][Finish_Selector]])
        Finish = Finish.flatten()
        return Start, Finish
    
    """
    Algorithm used to find most efficient way to final location
    
    Map is a boolean map that is true for valid roads and false for invalid areas
    Start_Loc is the starting location on the road network
    Finish_Loc is the finishing location on the road network
    """
    def A_Star(Map, Start_Loc, Finish_Loc):
        if Map[Start_Loc[0], Start_Loc[1]] == 0 or Map[Finish_Loc[0], Finish_Loc[1]] == 0:
            raise ValueError("Start and Finish Locations must fall on valid points on the map")
        #An Array used to store value of each point on map
        Value_Array = np.zeros((Map.shape))
        #Array used to verify if point has been checked (Code 9000000000 used to established checked areas) (Code 1000000000 used to mark unchecked areas)
        Check_Array = np.zeros((Map.shape))
        Check_Array[:, :] = np.inf
        #Array used to store current location being checked
        Current_Loc = np.zeros(2, dtype = int)
        Current_Loc[:] = Start_Loc[:]
        #Energy used to step in any given direction (Code 1000000000 is used as it is overly )
        Step_Array = np.array([[1.414, 1, 1.414],
                               [1, np.inf, 1],
                               [1.414, 1, 1.414]])
        #An array used to store the previous adjacent location point from which it's value was calculated
        Previous_Array = np.zeros((Map.shape[0], Map.shape[1], 2), dtype = int)
        
        #An array used to display the final path on the map
        Final_Path_Array = np.zeros((Map.shape))
        Final_Path_Array[:, :] = np.nan
        
        #Set the initial position to checked code
        Check_Array[Current_Loc[0], Current_Loc[1]] = 9000000000
        
        #Array used to trace from finsh to start using Previous_Array
        Trace_Loc = np.zeros(2, dtype = int)
        Trace_Loc[:] = Finish_Loc[:]
        
        #List used to store steps from finish to start
        Directions = []
        Checked_List = []
        
        #Calculate the values of valid points on the map until the finishing point has been found
        while not np.array_equal(Current_Loc, Finish_Loc):
            #For each 1 step in lat and long direction, Calculate the value of the new point on map
            Checked_List.append(Current_Loc.copy())
            for Latitude in range(-1, 2):
                for Longnitude in range(-1, 2):
                    #Value of Current Location
                    Current_Value = Value_Array[Current_Loc[0], Current_Loc[1]]
                    #Steps taken placed into array
                    Steps = np.array([Latitude, Longnitude])
                    #New location to have value calculated
                    Calc_Location = Current_Loc + Steps
                    #Energy required to make step to new location
                    Step_Energy = Step_Array[Latitude + 1, Longnitude + 1]
                    #Distance from finish
                    Distance_Value = np.sum(np.square(Calc_Location - Finish_Loc))
                    #A_Star Calculated Value
                    Calc_Value = Current_Value + Step_Energy + Distance_Value
                    #If new point is valid and unchecked and Calc_Value is lower than current value at new point, replace relevant points in Check, Value and Previous Array with relevant info
                    if Map[Calc_Location[0], Calc_Location[1]] != 0:
                        if Check_Array[Calc_Location[0], Calc_Location[1]] != 9000000000:
                            if Check_Array[Calc_Location[0], Calc_Location[1]] > Calc_Value:
                                Value_Array[Calc_Location[0], Calc_Location[1]] = Calc_Value
                                Check_Array[Calc_Location[0], Calc_Location[1]] = Calc_Value
                                Previous_Array[Calc_Location[0], Calc_Location[1], 0] = Current_Loc[0]
                                Previous_Array[Calc_Location[0], Calc_Location[1], 1] = Current_Loc[1]
            #Find point with minimum value
            Minimum_Location = np.unravel_index(Check_Array.argmin(), Check_Array.shape)
            #Set Current Location to Minimum_Location
            Current_Loc[0] = Minimum_Location[0]
            Current_Loc[1] = Minimum_Location[1]
            #Set Check Array of Minimum_Location to checked code in check array
            Check_Array[Current_Loc[0], Current_Loc[1]] = 9000000000
        #Retrace steps from finish to start and log directions and set Final_Path_Array to 1 along path
        while not np.array_equal(Trace_Loc, Start_Loc):
            Final_Path_Array[Trace_Loc[0], Trace_Loc[1]] = 1
            Temp_Trace_Loc_x = Previous_Array[Trace_Loc[0], Trace_Loc[1], 0]
            Temp_Trace_Loc_y = Previous_Array[Trace_Loc[0], Trace_Loc[1], 1]
            Trace_Loc[0] = Temp_Trace_Loc_x
            Trace_Loc[1] = Temp_Trace_Loc_y
            Directions.append(Trace_Loc.copy())
        return Check_Array, Final_Path_Array, Directions, Checked_List
        
class Graphics:
    """
    Display input array against UK map
    
    Graph Array is the array used to overlay a colour mesh over a road network image
    """
    def UK_Graph(Graph_Array):
        plt.figure(figsize=(16, 16))
        img = plt.imread("UK_Roads_Light.png")
        plt.imshow(img)
        plt.pcolormesh(np.flipud(Graph_Array), cmap="bwr", vmin = np.amin(Graph_Array), vmax = np.amin(Graph_Array))
        return plt
    
    """
    Creates an animation of checked areas on the map and finalises by overlaying the final route
    
    Animation_List is the order of areas checked by the algorithm
    Final_Graph is overlay array of the optimal route from start to finsih
    Video_Name is the file name of the animation to be saved
    """
    def UK_Graph_Animation(Animation_List, Final_Graph, Video_Name):
        Animation_Array = np.zeros(Road_Map.shape)
        Count = 0
        filenames = []
        for Step in Results[3]:
            Animation_Array[Step[0], Step[1]] = 1
            Count += 1
            if Count % 100 == 0:
                Filtered_Animation_Array = Animation_Array
                Filtered_Animation_Array[Filtered_Animation_Array == 0] = np.nan
                plt = Graphics.UK_Graph(Filtered_Animation_Array)
                plt.savefig("Pics/" + str(Count) + ".png")
                filenames.append("Pics/" + str(Count) + ".png")
                plt.close('all')

        plt = Graphics.UK_Graph(Filtered_Animation_Array)
        plt.pcolormesh(np.flipud(Final_Graph), cmap="bwr", vmin = 0, vmax = 1)  
        
        for Buffer_Frame in range(0, 30):
            Count += 1
            plt.savefig("Pics/" + str(Count) + ".png")
            filenames.append("Pics/" + str(Count) + ".png")
        plt.close('all')
        
        with imageio.get_writer(Video_Name, mode='I', fps = 30) as writer:
            for filename in filenames:
                image = imageio.imread(filename)
                writer.append_data(image)
            writer.close()
        for filename in set(filenames):
            os.remove(filename)
        


if __name__ == '__main__':
    Road_Map = Path_Finder.Data_Import()
    Locs = Path_Finder.Random_Start_End(Road_Map)
    Results = Path_Finder.A_Star(Road_Map, Locs[0], Locs[1])
    Graphics.UK_Graph(Results[0])
    Graphics.UK_Graph(Results[1])
    Graphics.UK_Graph_Animation(Results[3], Results[1], "RouteProcess.mp4")