/**** old line array to clear layers ***/
const oldLineArr = [];



const addLayer = (layer, addToOldLineArr = true) => {
    MAP.addLayer(layer);
    if (addToOldLineArr) {
        oldLineArr.push(layer);
    }
};

const clearLayer = (layer) => {
    MAP.removeLayer(layer);
};

const createLayerVector = (features) => {
    return new ol.layer.Vector({
        source: new ol.source.Vector({
            features: features
        })
    });
};

const clearOldLayers = () => {
    if (oldLineArr.length > 0) {
        oldLineArr.forEach(item => clearLayer(item));
        oldLineArr.splice(0, oldLineArr.length);
    }
};
/************* --------------------- *************/

/**** CONSTANTS ******/
const GROUP_NAME = "lbs2025";
const BASE_URL = "http://127.0.0.1:8000"; // when running locally
// const BASE_URL = "https://geonet.igg.uni-bonn.de"; // when uploading to server
const LEFT_BOTTOM = ol.proj.transform([7.004813999686911, 50.67771640948173], "EPSG:4326", "EPSG:3857");
console.log(LEFT_BOTTOM);
const RIGHT_TOP = ol.proj.transform([7.19776199427912, 50.768218129933224], "EPSG:4326", "EPSG:3857");
/**** -------- *****/


/**** MIN_MAX_COORDS *****/
const MIN_X = LEFT_BOTTOM[0];
const MIN_Y = LEFT_BOTTOM[1];
const MAX_X = RIGHT_TOP[0];
const MAX_Y = RIGHT_TOP[1];
/**** -------- *****/


/****** MARKER ICONS ******/
const FIRST_MARKER_ICON = new ol.style.Style({
    // The marker fetching from open source web service. If it is not working, please change the image link
    // you can add any png image from web ( Don't Forget to set the scale )
    image: new ol.style.Icon({
        crossOrigin: 'anonymous',
        src: 'https://cdn1.iconfinder.com/data/icons/web-55/32/web_1-1024.png',
        scale: "0.03"
    }),
});

const SECOND_MARKER_ICON = new ol.style.Style({
    // The marker fetching from open source web service.  If it is not working, please change the image link
    // you can add any png image from web ( Don't Forget to set the scale )
    image: new ol.style.Icon({
        crossOrigin: 'anonymous',
        src: 'https://cdn4.iconfinder.com/data/icons/twitter-29/512/157_Twitter_Location_Map-1024.png',
        scale: "0.04"
    }),
});
/**** -------- *****/

/****** MARKERS ******/
const FIRST_MARKER = new ol.Feature({
    geometry: new ol.geom.Point(1,1)
});
FIRST_MARKER.setStyle(FIRST_MARKER_ICON);

const SECOND_MARKER = new ol.Feature({
    geometry: new ol.geom.Point(1,1)
});
SECOND_MARKER.setStyle(SECOND_MARKER_ICON);
/**** -------- *****/


/***** INPUT ELEMENTS *****/
const firstCoordInput = document.getElementById("coord1");
const secondCoordInput = document.getElementById("coord2");
/**** -------- *****/


/****** LAYERS ******/
const TILE_LAYER = new ol.layer.Tile({ source: new ol.source.OSM()});
/**** -------- *****/


/**** MAP ****/
const MAP = new ol.Map({
    target: 'map',
    layers: [
        TILE_LAYER
    ],
    view: new ol.View({
        center: ol.proj.fromLonLat([(MIN_X + MAX_X)/2, (MAX_Y + MIN_Y)/2]),
        zoom: 2,
        maxZoom: 20,
        minZoom: 2,
        extent: [MIN_X, MIN_Y, MAX_X, MAX_Y],
    })
});
/**** -------- *****/

/***** PRESET STYLE *****/
const SHORTEST_PATH_STYLE = new ol.style.Style({
    stroke: new ol.style.Stroke({
        color: '#0080ff',
        width: 4,
        opacity: 1,
        lineDash: [.1, 7]
    })
});
const ALTERNATIVE1_PATH_STYLE = new ol.style.Style({
    stroke: new ol.style.Stroke({
        color: '#ff0000',
        width: 4,
        opacity: 1,
        lineDash: [.1, 7]
    })
});
const ALTERNATIVE2_PATH_STYLE = new ol.style.Style({
    stroke: new ol.style.Stroke({
        color: '#bf00ff',
        width: 4,
        opacity: 1,
        lineDash: [.1, 7]
    })
});
const STYLES = [SHORTEST_PATH_STYLE, ALTERNATIVE1_PATH_STYLE, ALTERNATIVE2_PATH_STYLE];
/**** -------- *****/

/***** VARIABLES *****/
let counter = 0;
let firstMarkerLayer;
let secondMarkerLayer;
let routeMethod = "walking";
let pathMethod = "shortestpath";
/**** -------- *****/

function drawPaths(path_info) {
    clearOldLayers();

    // Clear previous legend entries
    const legendList = document.getElementById('legend-list');
    legendList.innerHTML = '';

    for (let i = 0; i < path_info.length; i++) {
        let lineString = new ol.geom.LineString(path_info[i].path);
        let lineFeature = new ol.Feature({ geometry: lineString });
        let length = path_info[i].length;

        // Set Style for the Path
        lineFeature.setStyle(STYLES[i]);
        const line = createLayerVector([lineFeature]);
        addLayer(line);

        // Add Legend Entry with Distance
        const legendItem = document.createElement('li');
        legendItem.className = 'legend-item';
        legendItem.innerHTML = `
            <span style="width: 20px; height: 5px; background-color: ${STYLES[i].getStroke().getColor()}; display: inline-block; margin-right: 5px;"></span> 
            ${i === 0 ? 'Shortest Path' : `Alternative Path ${i}`} - ${length}`;
        legendList.appendChild(legendItem);
    }
}

/**** EVENT LISTENERS ****/



MAP.on("click", function (e) {
    clearOldLayers();

    const position = ol.proj.toLonLat(e.coordinate);

    if (counter%2 === 0) {
        clearLayer(secondMarkerLayer);
        FIRST_MARKER.getGeometry().setCoordinates(e.coordinate);
        firstMarkerLayer = createLayerVector([FIRST_MARKER]);
        addLayer(firstMarkerLayer, false);
        firstCoordInput.value = position[0].toFixed(7) + "," + position[1].toFixed(7);
        counter++;
    } else if (counter%2 === 1) {
        SECOND_MARKER.getGeometry().setCoordinates(e.coordinate);
        secondMarkerLayer = createLayerVector([SECOND_MARKER]);
        addLayer(secondMarkerLayer, false);
        secondCoordInput.value = position[0].toFixed(7) + "," + position[1].toFixed(7);
        counter++;
    }
});
/************* --------------------- *************/


/**** METHODS ******/

// Function to toggle button state and loader
const toggleLoadingState = (isLoading) => {
    const findPathBtn = document.getElementById("findPathBtn");
    const loadingIndicator = document.getElementById("loadingIndicator");

    if (isLoading) {
        findPathBtn.disabled = true;
        findPathBtn.style.opacity = "0.6";
        loadingIndicator.style.display = "inline-block";
    } else {
        findPathBtn.disabled = false;
        findPathBtn.style.opacity = "1";
        loadingIndicator.style.display = "none";
    }
};

async function findPath() {
    clearOldLayers();

    const [lon1, lat1] = firstCoordInput.value.split(",").map(parseFloat);
    const [lon2, lat2] = secondCoordInput.value.split(",").map(parseFloat);

    const requestBody = {
        lat1,
        lon1,
        lat2,
        lon2,
        isWheelChair: document.getElementById('isWheelChair').checked,
        isShortestPath: !document.getElementById('isAlternatePathBtn').checked,
        distance_weight: 1 - parseFloat(document.getElementById("slopeRange").value) / 100,
        epsilon: parseFloat(document.getElementById("lengthRestrictionRange").value) / 100,
        gamma: parseFloat(document.getElementById("sharedLengthRestrictionRange").value) / 100,
        alpha: parseFloat(document.getElementById("localOptimalityRange").value) / 100,
        fast_mode: document.getElementById('fastMode').checked
    };

    const url = `${BASE_URL}/get-path/`;

    console.log("Sending request to:", url, "with body:", requestBody);

    // Start loading state
    toggleLoadingState(true);

    try {
        const response = await fetch(url, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(requestBody)
        });

        const data = await response.json();
        drawPaths(data);
    } catch (error) {
        console.error("Error fetching path:", error);
    } finally {
        // End loading state
        toggleLoadingState(false);
    }
}

function updateSliderValue(sliderId, outputId) {
    const slider = document.getElementById(sliderId);
    const output = document.getElementById(outputId);
    output.textContent = slider.value/100;

    slider.addEventListener('input', () => {
        output.textContent = slider.value/100;
    });
}

// Tüm slider'lar için uygula
updateSliderValue('slopeRange', 'slopeValue');
updateSliderValue('localOptimalityRange', 'alphaValue');
updateSliderValue('sharedLengthRestrictionRange', 'gammaValue');
updateSliderValue('lengthRestrictionRange', 'epsilonValue');
