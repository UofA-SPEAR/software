import React, { Component } from 'react';
import ReactDOM from 'react-dom';
import GoogleMapReact from 'google-map-react';
import markerImage from './m1.png';
import ROSLib from 'roslib';
import {API_KEY} from './key';


class Marker extends Component {
    render() {
        return (
            <div style={{color: 'white', position: 'absolute', transform: 'translate(-50%, -100%)'}}>
                {this.props.text}
                <img src={markerImage} alt={this.props.text} style={{maxWidth: 50, maxHeight: 50, filter: "invert(100%) "}}></img>
            </div>
        )
    }
}

class SimpleMap extends Component {
    static defaultProps = {
        center: {
            lat: 51.45304,
            lng: -112.71593,
        },
        /* Starting position: 51.45304N 112.71593W
First flag: 51.45334N 112.71619W
Second flag: 51.45372N 112.71669W
Third flag: 51.45409N 112.71646W
Astronaut: 51.45380N 112.71619W
*/
        markers: [
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
                lat: 51.45380,
                lng: -112.71619,
                text: "Astronaut",
            },
            
        ],
        zoom: 20
    };

    constructor(props) {
        super(props);
        let ros = new ROSLib.Ros({url: 'ws://localhost:9090'});
        let gpsListener = new ROSLib.Topic({
            ros: ros,
            name: '/gps/fix',
            messageType: 'sensor_msgs/NavSatFix',
        });
        this.state = {
            ros,
            gpsListener,
            roverLatitude: this.props.center.lat,
            roverLongitude: this.props.center.lng,
        };
        this.state.gpsListener.subscribe(message => {
            console.log(message);
            this.setState({
                roverLatitude: message.latitude,
                roverLongitude: message.longitude,
            });
            this.forceUpdate();
        });
    }

    render() {
        const key = API_KEY;
        const markerComponents = this.props.markers.map(marker => {
            return (
                <Marker lat={marker.lat} lng={marker.lng} text={marker.text}></Marker>
            );
        });
        console.log(markerComponents);
        return (
            <div style={{ height: '100vh', width: '100%' }}>
                <GoogleMapReact
                bootstrapURLKeys={{ key: key }}
                defaultCenter={this.props.center}
                defaultZoom={this.props.zoom}
                yesIWantToUseGoogleMapApiInternals
                onGoogleApiLoaded={({ map, maps }) => this.handleApiLoaded(map, maps)}
                >
                    {markerComponents}
                    <Marker lat={this.state.roverLatitude} lng={this.state.roverLongitude} text="Rover"></Marker>
                </GoogleMapReact>
            </div>
        );
    }

    handleApiLoaded(map, maps) {
        map.setMapTypeId('hybrid');
    }

}

ReactDOM.render(
    (
        <SimpleMap />
    ),
    document.getElementById('root')
);