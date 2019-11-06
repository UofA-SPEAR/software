function load() {
    //var all = {};
    //all['test'] = '3';
    // This Big Brain: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html
    //localizaion status = https://developers.google.com/maps/documentation/javascript/geolocation

    //  sensor_msgs/BatteryState

    // language=JSON
    let modulesJSON = `
    [
        {
            "title": "Battery Voltage",
            "type": "number range",
            "yellow-range": [2.0,5.0],
            "red-range": [0.0,2.0],
            "topic": "web-test-battery",
            "message-type": "std_msgs/Float64"
        },
        {
            "title": "CAN Node Status",
            "type": "fraction",
            "topic": "web-can-status",
            "message-type": "std_msgs/String"
        },
        {
            "title": "Localization Status",
            "type": "gps",
            "topic": "web-local-status",
            "message-type": "std_msgs/String"
        },
        {
            "title": "Arm Joint Angles",
            "type": "number",
            "topic": "web-angles",
            "message-type": "std_msgs/Float64"
        },
        {
            "title": "Fuse Status",
            "type": "true false",
            "topic": "web-fuse-status",
            "message-type": "std_msgs/Bool"
        },
        {
            "title": "Motor Voltages",
            "type": "number",
            "topic": "web-motor",
            "message-type": "std_msgs/Float64"
        },
        {
            "title": "Power Usage",
            "type": "number",
            "topic": "web-power",
            "message-type": "std_msgs/Float64"
        },
        {
          "title": "Camera",
          "type": "video feed",
          "fps-topic": "web-fps-video1",
          "topic": "web-video1",
          "message-type": "sensor_msgs/Image"
        }
    ]
    `;

    //Maybe for later:
    //     {
    //         "type": "number range",
    //         "rangeType": "line"
    //     }

    var modules = JSON.parse(modulesJSON);

    var ros = new ROSLIB.Ros({
        url : URL
    });

    //console.log(modules);
    for (let x=0; x < modules.length; x++) {

        //Create topic here

        let tempTopic = new ROSLIB.Topic({
            ros : ros,
            name : modules[x].topic,
            messageType: modules[x]['message-type']

        });
        //console.log(modules[x]['message-type']);
        /*
        if (modules[x]['type'] === 'number range') {

        }
        */
        let htmlCard = `
            <div class="card" id="module-` + x + `-card" onclick="ScrollElement('module-extended-` + x + `-card')">
                <div class="module-container">
                    <h2 class="module-title">`+ modules[x].title + `</h2>
                    <p id="module-` + x + `-value">` + modules[x].type.toUpperCase() + `</p>
                </div>
            </div>
        `;
        let ExtCard = `
            <div class="card extended-card" id="module-extended-` + x + `-card" onclick="ScrollElement('module-` + x + `-value')">
                <div class="module-container">
                    <h2 class="module-title">`+ modules[x].title + `</h2>
                    <p id="module-extended-` + x + `-value">` + modules[x].type.toUpperCase() + `</p>
                </div>
            </div>
        `;

        document.getElementById('container-modules').innerHTML += htmlCard;
        document.getElementById('container-expanded-modules').innerHTML += ExtCard;




        tempTopic.subscribe(
            function(message) {
                document.getElementById('module-'+x+'-value').innerHTML = message.data;
                if (modules[x]['type'] === 'number range') {

                    if (modules[x]['red-range'][1] >= message.data && modules[x]['red-range'][0] < message.data) {
                        document.getElementById('module-'+x+'-card').style.backgroundColor = 'red'
                    } else if (modules[x]['yellow-range'][1] >= message.data && modules[x]['yellow-range'][0] < message.data) {
                        document.getElementById('module-'+x+'-card').style.backgroundColor = 'yellow'
                    } else {
                        document.getElementById('module-'+x+'-card').style.backgroundColor = 'green'
                    }
                    /*
                    if (modules[x]['red-range'][0] < message.data < modules[x]['red-range'][1]) {
                        //console.log("I DID A THING! " + message.data)
                    }
                    */


                }
            }
        );


        //Subscribe to it here
        //Push topic to list
        window.modulesLi.push(tempTopic);


        console.log(modules[x].type);
    }
    //console.log(document.getElementById('container-modules').innerHTML)
}