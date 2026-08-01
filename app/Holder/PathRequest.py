from pydantic import BaseModel


class PathRequest(BaseModel):
    """Variables of the frontend"""
    lat1: float
    lon1: float
    lat2: float
    lon2: float
    isWheelChair: bool
    isShortestPath: bool
    distance_weight: float 
    epsilon: float
    gamma: float
    alpha: float
    fast_mode: bool

