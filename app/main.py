from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

from .Holder.GraphHolder import GraphHolder
from .Holder.PathRequest import PathRequest
from .Service.Navigation import (
    shortest_path_by_wheelchair,
    shortest_path_by_walking,
    alternative_paths_by_walking,
    alternative_paths_by_wheelchair
)

# FastAPI app initialization
app = FastAPI()

# CORS settings
origins = [
    "http://localhost",
    "http://localhost:63342",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],# origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount static files (frontend) - this must be done AFTER defining API routes
# so that /get-path/ and other API routes take precedence
@app.post("/get-path/")
async def get_path(request: PathRequest):
    print(f"Request received: {request}")

    # TODO: Change these default parameter values
    request.alpha = 0.25      # local_optimality
    request.gamma = 0.65      # shared_edges_restriction (Shared edge length can be at most 65% of the shortest path length)
    request.epsilon = 0.25    # length_restriction (Length of the alternative path can be higher than the shortest path length by at most 25%)
    try:
        if request.isShortestPath:
            return await process_shortest_path(request)
        else:
            return await process_alternative_path(request)
    except Exception as e:
        print(f"ERROR: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# Process shortest path requests
async def process_shortest_path(request: PathRequest):
    try:
        if request.isWheelChair:
            return shortest_path_by_wheelchair(request)
        else:
            return shortest_path_by_walking(request)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


# Process alternative path requests
async def process_alternative_path(request: PathRequest):
    try:
        if request.isWheelChair:
            # Sensible adjustments to the parameters for wheelchair paths
            request.alpha = 0.20
            request.gamma = 0.70
            request.epsilon = 0.35
            return alternative_paths_by_wheelchair(request, fast_mode=request.fast_mode)
        else:
            return alternative_paths_by_walking(request, fast_mode=request.fast_mode)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


# Mount static files (frontend) at the END so API routes take precedence
frontend_path = Path(__file__).parent / "frontend"
if frontend_path.exists():
    app.mount("/", StaticFiles(directory=str(frontend_path), html=True), name="frontend")