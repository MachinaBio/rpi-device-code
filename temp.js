var i2c = require('i2c');
var address = 0x48;
var wire = new i2c(address, { device: '/dev/i2c-1' });

wire.readBytes(0, 2, function readBytes (error, response) {
  console.log(response);
  console.log('length:', response.length);
  console.log('Index 0:', response[0]);
  console.log('Index 1:', response[1]);

  var tens = response[0];
  var decimal = response[1] / 256;

  var result = tens + decimal;

  console.log('Celsius:', result);
});
