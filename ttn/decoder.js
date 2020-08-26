function Decoder(bytes, port) {
  var decoded = {};

  if (bytes.length == 20) {
    decoded.t = ((bytes[0]) | (bytes[1] << 8 ) | (bytes[2] << 16 ) | (bytes[3] << 24)) / 100.0;
    decoded.h = ((bytes[4]) | (bytes[5] << 8 ) | (bytes[6] << 16 ) | (bytes[7] << 24)) / 100.0;
    decoded.p = ((bytes[8]) | (bytes[9] << 8 ) | (bytes[10] << 16 ) | (bytes[11] << 24)) / 100.0;
    decoded.ppm10 = ((bytes[12]) | (bytes[13] << 8 ) | (bytes[14] << 16 ) | (bytes[15] << 24)) / 10.0;
    decoded.ppm25 = ((bytes[16]) | (bytes[17] << 8 ) | (bytes[18] << 16 ) | (bytes[19] << 24)) / 10.0;
    
  }

  return decoded;
}
