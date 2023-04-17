from abc import ABC, abstractmethod
import dataclasses
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np 

from sensor import Sensor 
from vehicle import Vehicle
from typedefs import ndarray

normalize: callable = lambda x: x / np.linalg.norm(x)

@dataclasses.dataclass 
class Wall: 
    endpoints: ndarray 
    inside_normal: ndarray

    def ray_intersection(self, ray_origin: ndarray, ray_direction: ndarray) -> list: 
        ray_direction = normalize(ray_direction)
        
        v1: ndarray = ray_origin - self.endpoints[0]
        v2: ndarray = self.endpoints[1] - self.endpoints[0]
        v3: ndarray = np.array([-ray_direction[1], ray_direction[0]])


        if np.dot(v2, v3) != 0.: 
            t1: ndarray = np.cross(v2, v1) / np.dot(v2, v3)
            t2: ndarray = np.dot(v1, v3) / np.dot(v2, v3)
        else: 
            t1 = -np.inf
            t2 = -np.inf

        intersections: List[ndarray] = []

        if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
            intersections.append(ray_origin + t1 * ray_direction)

        return intersections

    def translate(self, translation: np.ndarray) -> None: 
        self.endpoints += translation

    def draw(self, ax) -> None: 
        ax.plot(self.endpoints[0], self.endpoints[1], c="k")

class Environment(ABC): 
    @abstractmethod 
    def inside(self, point: ndarray) -> bool: 
        raise NotImplementedError

    @abstractmethod 
    def distance_to_boundary(self, point: ndarray, direction: ndarray) -> float: 
        raise NotImplementedError

    @abstractmethod 
    def draw(self, ax) -> None: 
        raise NotImplementedError

class BoxEnvironment(Environment): 
    def __init__(self, wall_length: Optional[float]=5.0) -> None: 
        self.wall_length: float = wall_length

        bottom_left: ndarray = np.array([-wall_length / 2., -wall_length / 2.])
        bottom_right: ndarray = np.array([wall_length / 2., -wall_length / 2.])
        top_right: ndarray = np.array([wall_length / 2., wall_length / 2.])
        top_left: ndarray = np.array([-wall_length / 2., wall_length / 2.])
        self.origin = np.zeros(2)

        up: ndarray = np.array([0., 1.])
        down: ndarray = np.array([0., -1.])
        right: ndarray = np.array([1., 0.])
        left: ndarray = np.array([-1., 0.])

        self._walls: Sequence[Wall] = [
            Wall(endpoints=np.array([bottom_left, bottom_right]), inside_normal=up), 
            Wall(endpoints=np.array([bottom_right, top_right]), inside_normal=left), 
            Wall(endpoints=np.array([top_right, top_left]), inside_normal=down), 
            Wall(endpoints=np.array([top_left, bottom_left]), inside_normal=right), 
        ]

    def translate(self, translation: np.ndarray) -> None: 
        self.origin = translation 
        for wall in self._walls: 
            wall.translate(translation)

    def __repr__(self) -> str: 
        return f"{self.__class__.__name__}(wall_length={self.wall_length})"

    def inside(self, point: ndarray) -> bool: 
        half_wall_length: float = self.wall_length / 2. 
        within_width: bool = (((point[0] - self.origin[0]) < half_wall_length) and ((point[0] - self.origin[0]) > -half_wall_length))
        within_height: bool = (((point[1] - self.origin[1]) < half_wall_length) and ((point[1] - self.origin[1]) > -half_wall_length))
        return (within_width and within_height)

    def distance_to_boundary(self, point: ndarray, direction: ndarray) -> float: 
        distances: list = [] 
        for wall in self._walls: 
            intersection = wall.ray_intersection(point, direction)

            if len(intersection) != 0: 
                distances.append(np.linalg.norm(intersection[0] - point))

        if len(distances) > 0: 
            return min(distances)
        else: 
            return [np.inf]

    def draw(self, ax) -> None: 
        for wall in self._walls: 
            wall.draw(ax)


class CompositeEnvironment(Environment): 
    def __init__(self, boxes: Sequence[BoxEnvironment], box_origins: Sequence[np.ndarray], exterior_wall_length: float) -> None: 
        self.exterior: BoxEnvironment = BoxEnvironment(exterior_wall_length)
        self.obstacles: Sequence[BoxEnvironment] = boxes 

        for obstacle, translation in zip(self.obstacles, box_origins): 
            obstacle.translate(translation)

    def inside(self, point: np.ndarray) -> bool: 
        is_inside: bool = self.exterior.inside(point)

        for obstacle in self.obstacles: 
            if obstacle.inside(point): is_inside = False

        return is_inside

    def distance_to_boundary(self, point: ndarray, direction: ndarray) -> float: 
        distance: float = self.exterior.distance_to_boundary(point, direction) 

        for obstacle in self.obstacles: 
            obstacle_distance: float = obstacle.distance_to_boundary(point, direction) 
            if (obstacle_distance < distance): distance = obstacle_distance

        return distance 

    def draw(self, ax) -> None: 
        self.exterior.draw(ax)
        for obstacle in self.obstacles: 
            obstacle.draw(ax)