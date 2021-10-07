This Howto describes how to build and run the Broadcom SPU kernel crypto driver.

SYSTEM SETUP
===================================================================

Kernel Configuration

	Make sure the following modules are enabled:
		Cryptographic API
			Software async crypto daemon (CONFIG_CRYPTO_CRYPTD)
			Authenc support (CONFIG_CRYPTO_AUTHENC)
			[M] Testing module (CONFIG_CRYPTO_TEST) (must be a
							       	module)
			CCM support (CONFIG_CRYPTO_CCM)
			GCM/GMAC support (CONFIG_CRYPTO_GCM)
			Sequence Number IV Generator (CONFIG_CRYPTO_SEQIV)
			CBC support (CONFIG_CRYPTO_CBC)
			CTR support (CONFIG_CRYPTO_CTR)
			ECB support (CONFIG_CRYPTO_ECB)
			XTS support (CONFIG_CRYPTO_XTS)
			HMAC support (CONFIG_CRYPTO_HMAC)
			CRC32c CRC algorithm (CONFIG_CRYPTO_CRC32C)
			GHASH digest algorithm (CONFIG_CRYPTO_GHASH)
			MD5 digest algorithm (CONFIG_CRYPTO_MD5)
			SHA1 digest algorithm (CONFIG_CRYPTO_SHA1)
			SHA224 and SHA256 digest algorithms
       						(CONFIG_CRYPTO_SHA256)
			SHA384 and SHA512 digest algorithms
       						(CONFIG_CRYPTO_SHA512)
			AES cipher algorithms (CONFIG_CRYPTO_AES)
			ARC4 cipher algorithm (CONFIG_CRYPTO_ARC4)
			DES and Triple DES EDE cipher algorithms
							(CONFIG_CRYPTO_DES)
			Pseudo Random Number Generation for Cryptographic
				       	modules (CONFIG_CRYPTO_ANSI_CPRNG)

		Kernel hacking --> Compile-time checks and compiler options
			Debug Filesystem (CONFIG_DEBUG_FS)


		        The SPU requires the Linux Mailbox framework and
			Broadcom FlexSparx Mailbox driver to communicate.
			To configure the kernel to include the Mailbox
			framework, include the following option:
			  Device Drivers --> Mailbox Hardware Support

			To enable the Broadcom FlexSparx Mailbox driver, which
		        uses the FA2 on Northstar+, the PDC on Northstar 2,
			and the MDE on Pegasus,	enable CONFIG_BCM_PDC_MBOX.
			This driver can be a module or built in.
			  Mailbox Hardware Support --> Broadcom FlexSparx
			  			       DMA Mailbox

			To enable the SPU driver, enable
			CONFIG_CRYPTO_DEV_BCM_SPU:
			Cryptographic API --> Hardware crypto devices -->
				Broadcom symmetric crypto/hash acceleration
				support


Then do a full build. Refer to LDK guide for how to build.

For user applications (including the cryptotest application), the
cryptodev-linux module is required.  If using brcm-dev-image, this module
is located at /lib/modules/<kernel_version>/extra/cryptodev.ko.  If using
another image, the source code can be found at http://cryptodev-linux.org/ .


USAGE
===========================================================================

On boot:
	modprobe bcm_crypto_spu
	modprobe cryptodev   (for /dev/crypto -based userspace applications)

When the SPU driver loads, it registers the set of algorithms it supports.
The algorithms that are registered are listed in

/proc/crypto

The SPU driver has debug tracing which can be enabled at runtime, if the
driver has been built with debugging enabled. See the SPU driver Makefile
to build with debug enabled. To enable or disable SPU driver debugging at
runtime, use the following commands, replacing <module_name> with the
appropriate string:

	# Turn on debug logging of execution flow
	echo "1" > /sys/module/bcm_crypto_spu/parameters/flow_debug_logging
	# Turn on debug logging of packet data in/out
	echo "1" > /sys/module/bcm_crypto_spu/parameters/packet_debug_logging

Set the above parameters to 0 to turn off debugging.

	# Add delays in the debugging to separate the flows and/or slow down
	# output
	# The delay is in msec units and is per normal line of debug and
	# and at the end of each hex dump. This option should not be enabled
	# when running IPsec traffic.
	# (Note: not normally used)
	echo "5" > /sys/module/bcm_crypto_spu/parameters/debug_logging_sleep

The SPU driver lists statistics in a debug FS entry. For example:

	# cat /sys/kernel/debug/bcm_crypto_spu/stats
	Number of SPUs.........4
	Number of channels.....4
	Current sessions.......0
	Session count..........19
	Cipher setkey..........140
	HMAC setkey............132
	AEAD setkey............91
	Cipher Ops.............140
	Hash Ops...............74
	HMAC Ops...............132
	AEAD Ops...............82
	Bytes of req data......38390
	Bytes of resp data.....26132
	Mailbox send failures..0
	Check ICV errors.......0
	SPU 0 output FIFO high water.....0
	SPU 1 output FIFO high water.....0
	SPU 2 output FIFO high water.....0
	SPU 3 output FIFO high water.....0

TESTING
===========================================================================

cryptotest is a userspace utility for testing crypto algorithms. To use
cryptotest, the following module must be loaded:

	modprobe cryptodev

Here are some sample tests using cryptotest:

	# print out usage info
	cryptotest --help
	# do a paired cbc-aes encrypt/decryp with verbose output
	cryptotest -v -a aes
	# same operation and check the final cleartext with original
	cryptotest -c -v -a aes
	# same operation but on a 1024 byte datablock (random).
	cryptotest -c -v -a aes 1 1024
	# same operation but performed 1000 times, new random data each time.
	cryptotest -c -v -a aes 1000 1024
	# same operation but all key/iv/data is 0x00. Quiet for timings
	cryptotest -Z -c -a aes 1000 1024
	# do 1 encrypt then 1000 decrypt operations - (U)nencrypt only, quiet
	cryptotest -U -a aes 1000 1024
	# do 1000 encrypt operations then 1 decrypt op - (E)ncrypt only, quiet
	cryptotest -E -a aes 1000 1024

	# generate a sha1 hash of a 160 byte datablock
	cryptotest -v -a sha1 1 160
	# generate an md5 hash of a 2k byte datablock 2000 times - quiet
	cryptotest -a md5 2000 2048

openssl can also be used to exercise the SPU.

	# use openssl to benchmark
	openssl speed -evp aes-128-cbc -engine cryptodev -elapsed
	# use openssl to run a single encrypt operation
	echo -n "@@@@@@@@@@@@@@@" | openssl aes-128-ecb -K 00000000000000000000000000000000 -iv 00000000000000000000000000000000 | hexdump -C

The Linux kernel includes a test module, called tcrypt, which can be used to
verify the correct operation of the SPU. To build the tcrypt module, enable
the kernel configuration option, CONFIG_CRYPTO_TEST, under

	Cryptographic API -->
		[M] Testing module

tcrypt tests are a no-op when the kernel configuration option
CRYPTO_MANAGER_DISABLE_TESTS is enabled. The kernel disables tests by default.
To enable tcrypt tests, turn off this option:

	Cryptographic API -->
		[ ] Disable run-time self tests

Note that when this option is turned off, the kernel automatically runs tests
for each crypto algorithm when it is registered, and if it fails, the kernel
marks it as failed and will not use it. Algorithms that fail self-tests at
registration time are marked as "unknown" in /proc/crypto. Algorithms that
pass self-tests are marked as "passed." (If self tests are disabled, all
algorithms are marked as passed.)

# more /proc/crypto
name         : hmac(sha512)
driver       : hmac-sha512-iproc
module       : kernel
priority     : 400
refcnt       : 1
selftest     : unknown
type         : ahash
async        : yes
blocksize    : 128
digestsize   : 64

Tests are run by loading the tcrypt module for a specific mode. A mode tests
a crypto algorithm. The module automatically exits after each test so that
another test can be run. If the test is successful, the module will exit
with the following "error":

	modprobe: can't load module tcrypt (kernel/crypto/tcrypt.ko): Resource temporarily unavailable

Any other errors are actual failures of some kind (possibly expected, i.e. for
unsupported things).  Other than the above error, silence is a positive result.

tcrypt includes both functional tests and speed tests. Some examples are as
follows:

	modprobe tcrypt mode=1   # md5
	modprobe tcrypt mode=2   # sha1
	modprobe tcrypt mode=6   # sha256
	modprobe tcrypt mode=3   # ecb(des), cbc(des)
	modprobe tcrypt mode=4   # ecb(3des_ede), cbc(3des_ede)
	modprobe tcrypt mode=10  # ecb(aes), cbc(aes), ctr(aes)
	modprobe tcrypt mode=16  # ecb(arc4)
	modprobe tcrypt mode=100 # hmac(md5)
	modprobe tcrypt mode=101 # hmac(sha1)
	modprobe tcrypt mode=102 # hmac(sha256)

	modprobe tcrypt mode=155 # authenc(cbc(aes),hmac(sha1|sha256))

	modprobe tcrypt mode=402 sec=1 # md5 speed tests
	modprobe tcrypt mode=403 sec=1 # sha1 speed tests
	modprobe tcrypt mode=404 sec=1 # sha256 speed tests

	modprobe tcrypt mode=500 sec=1 # aes speed tests
	modprobe tcrypt mode=501 sec=1 # 3des_ede speed tests
	modprobe tcrypt mode=502 sec=1 # des speed tests
	modprobe tcrypt mode=505 sec=1 # ecb(arc4) speed tests

See crypto/tcrypt.c for a complete list of tcrypt tests.

LIMITATIONS
===========================================================================
* AES-XCBC hashing is performed using the SPU hardware only when a full digest
  is requested.  When partial / incremental hashing is requested, the driver
  performs these in software, as incremental AES-XCBC hashing is not supported
  in the hardware.
* Hardware support for AES-CCM on the Northstar2 device supports authentication
  digest lengths of 8, 12, and 16 bytes only.  Crypto requests with other
  digest lengths will be performed in software.
* Hardware for AES-CCM on the Northstar Plus platform does not support AAD
  length of 0.  Crypto requests with this AAD length will be performed in
  software.
