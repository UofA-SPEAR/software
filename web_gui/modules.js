//var all = {};
//all['test'] = '3';
let modulesJSON = `
[
    {
        "title": "Battery Voltage",
        "type": "number range",
        "yellow-range": [2.0,5.0],
        "red-range": [0.0,2.0],
        "topic": "web-test-battery",
        "message-type": "std_msgs/Float64"
    }
]
`;

//Maybe for later:
//     {
//         "type": "number range",
//         "rangeType": "line"
//     }

let URL = 'ws://spear.northcentralus.cloudapp.azure.com:9090';
var modules = JSON.parse(modulesJSON);
var modulesLi = [];

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
        <div class="card" id="module-` + x + `-card">
            <div class="module-container">
                <h2 class="module-title">`+ modules[x].title + `</h2>
                <p id="module-` + x + `-value">{NUMBER RANGE}</p>
            </div>
        </div>
    `;

    document.getElementById('container-modules').innerHTML += htmlCard;



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

    modulesLi.push(tempTopic);


    console.log(modules[x].type);
}
