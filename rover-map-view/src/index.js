import GoogleMapReact from "google-map-react";
import React from "react";
import ReactDOM from "react-dom";
import ROSLib from "roslib";
import { API_KEY } from "./key";
import markerImage from "./m1.png";

function Marker({ text }) {
  return (
    <div
      style={{
        color: "white",
        position: "absolute",
        transform: "translate(-50%, -100%)",
      }}
    >
      {text}
      <img
        src={markerImage}
        alt={text}
        style={{ maxWidth: 50, maxHeight: 50, filter: "invert(100%)" }}
      ></img>
    </div>
  );
}

/**
 * @typedef {{lat: number, lng: number}} Coord
 * @typedef {Coord & {text: string}} Marker
 */

/**
 * @param {{markers: Marker[], defaultCenter: Coord, defaultZoom: number}} props
 */
function SimpleMap({ markers, defaultCenter, defaultZoom }) {
  const [roverPosition, setRoverPosition] = React.useState({
    ...defaultCenter,
  });

  React.useEffect(() => {
    const ros = new ROSLib.Ros({ url: "ws://localhost:9090" });
    const gpsListener = new ROSLib.Topic({
      ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix",
    });
    const onNavSatFix = (message) => {
      console.log(message);
      setRoverPosition({
        lat: message.latitude,
        lng: message.longitude,
      });
    };
    gpsListener.subscribe(onNavSatFix);

    return () => {
      gpsListener.unsubscribe(onNavSatFix);
      ros.close();
    };
  }, []);

  const markerComponents = React.useMemo(
    () =>
      markers.map((marker) => {
        return (
          <Marker
            key={marker.text}
            lat={marker.lat}
            lng={marker.lng}
            text={marker.text}
          ></Marker>
        );
      }),
    [markers]
  );

  const onGoogleApiLoaded = ({ map }) => {
    map.setMapTypeId("hybrid");
  };

  return (
    <div style={{ height: "100vh", width: "100%" }}>
      <GoogleMapReact
        bootstrapURLKeys={{ key: API_KEY }}
        defaultCenter={defaultCenter}
        defaultZoom={defaultZoom}
        yesIWantToUseGoogleMapApiInternals
        onGoogleApiLoaded={onGoogleApiLoaded}
      >
        {markerComponents}
        <Marker
          lat={roverPosition.lat}
          lng={roverPosition.lng}
          text="Rover"
        ></Marker>
      </GoogleMapReact>
    </div>
  );
}

const defaultCenter = {
  lat: 51.45304,
  lng: -112.71593,
};

const markers = [
  {
    lat: 51.45304,
    lng: -112.71593,
    text: "Starting position",
  },
  {
    lat: 51.45334,
    lng: -112.71619,
    text: "First flag",
  },
  {
    lat: 51.45372,
    lng: -112.71669,
    text: "Second flag",
  },
  {
    lat: 51.45409,
    lng: -112.71646,
    text: "Third flag",
  },
  {
    lat: 51.4538,
    lng: -112.71619,
    text: "Astronaut",
  },
];

const defaultZoom = 20;

ReactDOM.render(
  <SimpleMap
    defaultCenter={defaultCenter}
    markers={markers}
    defaultZoom={defaultZoom}
  />,
  document.getElementById("root")
);
