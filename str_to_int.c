static int read_file_int(const char *path, int *val)
{
	char buf[32];
	int ret;
	int tmp;
	char *end;

	ret = read_file(path, buf, sizeof(buf));
	if (ret < 0)
		return -1;

	tmp = strtol(buf, &end, 0);
	if (end == buf ||
			((end < buf+sizeof(buf)) && (*end != '\n' && *end != '\0')))
		goto err;

	*val = tmp;
	return 0;

err:
	return -1;
}


read_file_int(FAKE_BATTERY_CAPATITY_PATH, &nCapacity);
