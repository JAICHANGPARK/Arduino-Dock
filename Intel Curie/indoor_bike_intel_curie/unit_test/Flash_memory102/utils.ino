void dump(uint8_t *dt, uint32_t n)
{

  uint32_t sz;
  char buf[64];
  uint16_t clm = 0;
  uint8_t data;
  uint8_t sum;
  uint8_t vsum[16];
  uint8_t total = 0;
  uint32_t saddr = 0;
  uint32_t eaddr = n - 1;
  sz = eaddr - saddr;

  Serial.print("----------------------------------------------------------\n");
  for (uint16_t i = 0; i < 16; i++) {
    vsum[i] = 0;
  }
  for (uint32_t addr = saddr; addr <= eaddr; addr++) {
    data = dt[addr];
    if (clm == 0) {
      sum = 0;
      sprintf(buf, "%05lx: ", addr);
      Serial.print(buf);
    }

    sum += data; // sum = sum + data
    vsum[addr % 16] += data;

    sprintf(buf, "%02x ", data); // 입력된 데이터 파라미터
    Serial.print(buf);

    clm++;

    if (clm == 16) {
      sprintf(buf, "|%02x ", sum);
      Serial.print(buf);
      Serial.print("\n");
      clm = 0;
    }
  }
  Serial.print("----------------------------------------------------------\n");
  Serial.print("       ");
  for (uint16_t i = 0; i < 16; i++) {
    total += vsum[i];
    sprintf(buf, "%02x ", vsum[i]);
    Serial.print(buf);
  }
  sprintf(buf, "|%02x ", total);
  Serial.print(buf);
  Serial.print("");
  Serial.print("");
  Serial.print("\n");
}


