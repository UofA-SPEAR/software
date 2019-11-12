function numberRangeCallback(message, modules, x) {

    if (modules[x]['red-range'][1] >= message.data && modules[x]['red-range'][0] < message.data) {
        document.getElementById('module-' + x + '-card').style.backgroundColor = 'red'
    } else if (modules[x]['yellow-range'][1] >= message.data && modules[x]['yellow-range'][0] < message.data) {
        document.getElementById('module-' + x + '-card').style.backgroundColor = 'yellow'
    } else {
        document.getElementById('module-' + x + '-card').style.backgroundColor = 'green'
    }
}