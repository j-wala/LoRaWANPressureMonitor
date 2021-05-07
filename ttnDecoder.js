function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  // Decode 2 bytes to a signed integer. As the
  // bitwise operators in JavaScript expect 32
  // bits, this needs "sign extension" to support
  // negative values. Shifting 24 bits leftwards,
  // followed by shifting 16 bits to the right,
  // extends a "two's complement" negative value
  // such as 0xF6D4 into 0xFFFFF6D4.
  var pressureInt = (bytes[0] << 24 >> 16) | bytes[1];
  var batteryInt = (bytes[2] << 24 >> 16) | bytes[3];

  // Decode integer to float
  decoded.value = pressureInt / 1000;
  decoded.batteryVoltage = batteryInt;

  return decoded;
}
