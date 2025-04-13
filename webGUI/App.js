import logo from './logo.svg';
import './App.css';
import React, { useState, useEffect } from "react";
import io from "socket.io-client";
import { MapContainer, TileLayer, Marker, Popup, useMap } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import L from "leaflet";
import { CircularProgressbar, buildStyles } from "react-circular-progressbar";
import "react-circular-progressbar/dist/styles.css";
import baseStationIconImg from './BaseStation.png';

const socket = io("http://localhost:5000"); // WebSocket server address

const gpsIcon = new L.Icon({
    iconUrl: "/rover.png",
    iconSize: [20, 20],
    popupAnchor: [0, -40]
});

const baseStationIcon = new L.Icon({
    iconUrl: baseStationIconImg,
    iconSize: [35, 35],
    popupAnchor: [0, -40]
});

function ChangeView({ center }) {
    const map = useMap();
    map.setView(center, 20);
    return null;
}


function TemperatureGauge({ value }) {
    const gaugeColor = value > 82 ? "red" : "#FFA500"; // Change color at threshold
    return (
        <div style={{ width: "120px", height: "120px", margin: "50px auto 0 auto" }}>
            <CircularProgressbar
                value={value}
                text={`${value}°F`}
                styles={buildStyles({
                    textSize: "16px",
                    pathColor: gaugeColor,
                    textColor: gaugeColor,
                    trailColor: "#d6d6d6"
                })}
            />
        </div>
    );
}

function App() {
  const [gpsData, setGpsData] = useState("Waiting for GPS data...");
  const [gps, setGps] = useState({ lat: 0, lon: 0 });
  const [baseStation, setBaseStation] = useState({ lat: 45.381, lon: -73.495 }); // hardcoded initial location
  const [temperature, setTemperature] = useState(80); // hardcoded initial temp

    useEffect(() => {
        // Hardcoded GPS data for testing
        const testGpsData = "$GNGGA,022833.00,4522.90230,N,07329.75320,W,1,12,1.85,21.9,M,-32.6,M,,*44";
        setGpsData(testGpsData);

        // Parse data
        const testParts = testGpsData.split(",");
        if (testParts.length > 5 && testParts[2] && testParts[4]) {
            let latDegrees = parseInt(testParts[2].slice(0, 2), 10);
            let latMinutes = parseFloat(testParts[2].slice(2)) / 60;
            let latitude = latDegrees + latMinutes;

            let lonDegrees = parseInt(testParts[4].slice(0, 3), 10);
            let lonMinutes = parseFloat(testParts[4].slice(3)) / 60;
            let longitude = lonDegrees + lonMinutes;

            // check if hemisphere are good
            if (testParts[3] === "S") latitude = -latitude;
            if (testParts[5] === "W") longitude = -longitude;

            console.log(` Corrected GPS: Lat ${latitude}, Lon ${longitude}`);
            setGps({ lat: latitude, lon: longitude });
        }

        // Where we would take uart layer data, but turned off for testing
        // uncomment when needed  !!!!!!!!!!!!
        // socket.on("base_station_position", (data) => {
        //     setBaseStation({ lat: data.lat, lon: data.lon });
        // });

        // COMMENT OUT during real system
        // this randomly changes location for demo of system working
        const intervalId = setInterval(() => {
            setGps(prevGps => {
                const deltaLat = (Math.random() - 0.5) * 0.0005; // small change
                const deltaLon = (Math.random() - 0.5) * 0.0005; // small change
                return {
                    lat: prevGps.lat + deltaLat,
                    lon: prevGps.lon + deltaLon
                };
            });
        }, 1000);

        // COMMENT OUT during real system
        // this randomly changes Temperature for demo of system working
        const tempInterval = setInterval(() => {
            setTemperature(prevTemp => {
                const fluctuation = (Math.random() > 0.5 ? 1 : -1) * (Math.random() > 0.7 ? 1 : 0);
                return prevTemp + fluctuation;
            });
        }, 1000);

        return () => {
            socket.off("update_gps");
            clearInterval(intervalId);
            clearInterval(tempInterval);
        };
    }, []);

  return (
      <div style={{ textAlign: "center", padding: "20px" }}>
          <h1>Space Concordia Rover Dashboard</h1>
          <p>Latitude: {gps.lat.toFixed(6)}, Longitude: {gps.lon.toFixed(6)}</p>

          <div className='GUIContainer'>

          <MapContainer center={[gps.lat, gps.lon]} zoom={15} style={{ height: "500px", width: "80%", margin: "auto" }}>
              <ChangeView center={[gps.lat, gps.lon]} />
              <TileLayer
                  url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                  attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
              />
              <Marker position={[gps.lat, gps.lon]} icon={gpsIcon}>
                  <Popup>
                      🚀 GPS Position <br />
                      Lat: {gps.lat.toFixed(6)}, Lon: {gps.lon.toFixed(6)}
                  </Popup>
              </Marker>
              <Marker position={[baseStation.lat, baseStation.lon]} icon={baseStationIcon}>
                  <Popup>
                      🛰️ Base Station<br />
                      Lat: {baseStation.lat.toFixed(6)}, Lon: {baseStation.lon.toFixed(6)}
                  </Popup>
              </Marker>
          </MapContainer>
              <div className='TempContainer'>
                  <h2>Temperature</h2>
                  <TemperatureGauge value={temperature} />
              </div>
          </div>
      </div>

  )
}

export default App;