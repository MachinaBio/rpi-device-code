var i2c = require('i2c');
var address = 0x70;
var wire =  new i2c(address, { device: '/dev/i2c-1' });

var byte = parseInt(process.argv[2], 10);

  if (!byte) throw new Error('No byte defined to write!');

wire.readByte(function readByte (error, response) {
  if (error) throw error;

  console.log(response);

  wire.writeByte(byte, function writeBtye (error) {
    if (error) throw error;

    console.log('Byte written:', byte);

    wire.readByte(function readByte (error, response) {
      if (error) throw error;

      console.log(response);
    })
  });
});
