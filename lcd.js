var i2c = require('i2c');
var address = 0x28;
var wire = new i2c(address, { device: '/dev/i2c-1' });

var command = 0xFE;
var off = 0x42;
var on  = 0x41;
var clear = 0x51;
var home = 0x46;
var cursor = 0x4B;

function throwError (error) {
  if (error) throw error;
}

wire.writeBytes(0xFE, 0x4B, throwError);
//wire.writeBytes(0xFE, 0x42, throwError);
//wire.writeBytes(0xFE, 0x41, throwError);
//wire.writeBytes(0xFE, 0x51, throwError);
//wire.writeBytes(0xFE, 0x46);
//wire.writeBytes('It works!');
