

uint32_t readUInt32(unsigned char *buffer)
{
    return ((buffer[0] << 0) | (buffer[1] << 8) |
            (buffer[2] << 16) | (buffer[3] << 24));
}

void writeUInt32(unsigned char *buffer, uint32_t val)
{
    buffer[0] = val & 0xFF;
    buffer[1] = (val >> 8) & 0xFF;
    buffer[2] = (val >> 16) & 0xFF;
    buffer[3] = (val >> 24) & 0xFF;
}