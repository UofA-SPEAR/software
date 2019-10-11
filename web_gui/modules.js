//var all = {};
//all['test'] = '3';
let modulesJSON = `
[
    {
        "title": "Battery Voltage",
        "type": "number range",
        "yellow-range": [3,5],
        "red-range": [0,2],
        "topic": "web-test-battery"
    }
]
`;

//Maybe for later:
//     {
//         "type": "number range",
//         "rangeType": "line"
//     }


var modules = JSON.parse(modulesJSON);

console.log(modules);
for (let x=0; x < modules.length; x++) {



    if (modules[x]['type'] === 'number range') {

    }



    console.log(modules[x].type);
}
