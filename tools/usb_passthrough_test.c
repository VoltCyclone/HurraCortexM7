// usb_passthrough_test.c — test vendor/class request passthrough via libusb
// Usage: usb_passthrough_test <vid> <pid>
//   e.g. usb_passthrough_test 046d c539
//
// Build: cc -o usb_passthrough_test usb_passthrough_test.c \
//          $(pkg-config --cflags --libs libusb-1.0)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

static void hex_dump(const uint8_t *buf, int len)
{
	for (int i = 0; i < len; i++)
		printf("%02x ", buf[i]);
	printf("\n");
}

int main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "Usage: %s <vid> <pid>\n", argv[0]);
		fprintf(stderr, "  e.g. %s 046d c539\n", argv[0]);
		return 1;
	}

	uint16_t vid = (uint16_t)strtoul(argv[1], NULL, 16);
	uint16_t pid = (uint16_t)strtoul(argv[2], NULL, 16);

	libusb_context *ctx = NULL;
	int r = libusb_init(&ctx);
	if (r < 0) {
		fprintf(stderr, "libusb_init failed: %s\n", libusb_strerror(r));
		return 1;
	}

	libusb_device_handle *dev = libusb_open_device_with_vid_pid(ctx, vid, pid);
	if (!dev) {
		fprintf(stderr, "Device %04x:%04x not found\n", vid, pid);
		libusb_exit(ctx);
		return 1;
	}
	printf("Opened device %04x:%04x\n", vid, pid);

	// Only detach interface 0 — minimize disruption
	int detached = 0;
	if (libusb_kernel_driver_active(dev, 0) == 1) {
		printf("Detaching kernel driver on interface 0\n");
		libusb_detach_kernel_driver(dev, 0);
		detached = 1;
	}

	uint8_t buf[256];
	int transferred;

	// --- Test 1: GET_REPORT (class IN) ---
	// This tests the IN passthrough path (device reads from real device)
	printf("\n--- Test 1: GET_REPORT (class IN) ---\n");
	memset(buf, 0, sizeof(buf));
	transferred = libusb_control_transfer(dev,
		0xA1,       // bmRequestType: IN | class | interface
		0x01,       // bRequest: GET_REPORT
		0x0100,     // wValue: type=input(1), id=0
		0x0000,     // wIndex: interface 0
		buf, 64,
		1000);
	if (transferred >= 0) {
		printf("  OK: received %d bytes: ", transferred);
		hex_dump(buf, transferred);
	} else {
		printf("  FAILED: %s\n", libusb_strerror(transferred));
	}

	// --- Test 2: SET_REPORT (class OUT) ---
	// This tests the OUT passthrough path (computer sends data to real device)
	// Use report type=output(2), id=0, minimal 1-byte payload
	printf("\n--- Test 2: SET_REPORT (class OUT, 1 byte) ---\n");
	buf[0] = 0x00;
	transferred = libusb_control_transfer(dev,
		0x21,       // bmRequestType: OUT | class | interface
		0x09,       // bRequest: SET_REPORT
		0x0200,     // wValue: type=output(2), id=0
		0x0000,     // wIndex: interface 0
		buf, 1,
		1000);
	if (transferred >= 0) {
		printf("  OK: sent %d bytes\n", transferred);
	} else {
		printf("  STALL: %s (normal if device doesn't accept this report)\n",
			libusb_strerror(transferred));
		printf("  Key test: did the proxy stay alive? (mouse still works?)\n");
	}

	// --- Test 3: Vendor IN request ---
	printf("\n--- Test 3: Vendor IN request ---\n");
	memset(buf, 0, sizeof(buf));
	transferred = libusb_control_transfer(dev,
		0xC0,       // bmRequestType: IN | vendor | device
		0x01,       // bRequest: arbitrary
		0x0000, 0x0000,
		buf, 64,
		1000);
	if (transferred >= 0) {
		printf("  OK: received %d bytes: ", transferred);
		hex_dump(buf, transferred);
	} else {
		printf("  STALL: %s (normal — real device doesn't support this)\n",
			libusb_strerror(transferred));
	}

	// --- Test 4: Vendor OUT request ---
	printf("\n--- Test 4: Vendor OUT request with 4 bytes ---\n");
	buf[0] = 0xDE; buf[1] = 0xAD; buf[2] = 0xBE; buf[3] = 0xEF;
	transferred = libusb_control_transfer(dev,
		0x40,       // bmRequestType: OUT | vendor | device
		0x01,       // bRequest: arbitrary
		0x0000, 0x0000,
		buf, 4,
		1000);
	if (transferred >= 0) {
		printf("  OK: sent %d bytes\n", transferred);
	} else {
		printf("  STALL: %s (normal — real device doesn't support this)\n",
			libusb_strerror(transferred));
	}

	// Re-attach kernel driver so mouse keeps working
	if (detached) {
		printf("\nRe-attaching kernel driver on interface 0\n");
		libusb_attach_kernel_driver(dev, 0);
	}

	printf("\nResults:\n");
	printf("  Test 1 (GET_REPORT IN):  passthrough works if OK\n");
	printf("  Test 2 (SET_REPORT OUT): passthrough works if OK or STALL\n");
	printf("                           (STALL = real device rejected, proxy forwarded correctly)\n");
	printf("  Tests 3-4 (vendor):      same — STALL from real device is correct behavior\n");
	printf("  CRITICAL: proxy must stay alive after all tests (mouse still works)\n");

	libusb_close(dev);
	libusb_exit(ctx);
	return 0;
}
