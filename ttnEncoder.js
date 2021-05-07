// Implements an encoder for sending data to the Board (Not implemented on Board)
function Encoder(object, port) {
  // Encode downlink messages sent as
  // object to an array or buffer of bytes.
  var criticalValueMin = object["criticalValueMin"] * 1000;
  var criticalValueMax = object["criticalValueMax"] * 1000;
  var bytes = [];
  bytes[0] = (criticalValueMin & 0xFF00) >> 8;
  bytes[1] = (criticalValueMin & 0x00FF);
  bytes[2] = (criticalValueMax & 0xFF00) >> 8;
  bytes[3] = (criticalValueMax & 0x00FF);

  // if (port === 1) bytes[0] = object.led ? 1 : 0;

  return bytes;
}
