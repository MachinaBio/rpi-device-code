var rasp2c = require('rasp2c');
var address = 0x48;
var range = '0-2';

setTimeout(function () {
  rasp2c.dump(address, range, function (error, result) {
    if (error) throw error;

    console.log(result.toString());

  });
}, 1000);
