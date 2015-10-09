#!/bin/bash

ROOT="$PWD"

if [ ! -s "$ROOT/rustc-sysroot" ]; then
	echo $0 must be run from the root rust directory 1>&2
	return 1
fi

function rbuild_llvm() {
	(
		cd $ROOT &&
		[ ! -s "$ROOT/Makefile" ] && $ROOT/configure || true &&
		make -C $ROOT llvm &&
		rbuild_llvm_config
	)
}

function rbuild_llvm_config() {
	local LLVM_CONFIG="$1"
	if [ -z "$LLVM_CONFIG" ]; then
		LLVM_CONFIG=` echo "$ROOT/"*"/llvm/Release/bin/llvm-config"`
	fi

	if [ ! -e "$LLVM_CONFIG" ]; then
		echo "$LLVM_CONFIG not found" 1>&2
		return 1
	fi

	export CFG_LLVM_CONFIG="$LLVM_CONFIG"
	rbuild_host `"$CFG_LLVM_CONFIG" --host-target`
	if [ -z "$TARGET" ]; then
		rbuild_target $CFG_COMPILER_HOST_TRIPLE
	fi
}

function rbuild_git_cfg() {
	local CFG_GIT_DIR="$ROOT/.git"

	export CFG_RELEASE_NUM=`grep '^CFG_RELEASE_NUM=' "$ROOT/mk/main.mk" | cut -d '=' -f 2`
	export CFG_RELEASE="$CFG_RELEASE_NUM-dev"
	if [ -d "$CFG_GIT_DIR" ]; then
		export CFG_VER_DATE=`git --git-dir="$CFG_GIT_DIR" log -1 --date=short --pretty=format:'%cd'`
		export CFG_VER_HASH=`git --git-dir="$CFG_GIT_DIR" rev-parse HEAD`
		export CFG_SHORT_VER_HASH=`git --git-dir="$CFG_GIT_DIR" rev-parse --short=9 HEAD`
	else
		export CFG_VER_DATE=`date --rfc-3339=date`
	fi
	export CFG_VERSION="$CFG_RELEASE ($CFG_SHORT_VER_HASH $CFG_VER_DATE)"
}

function rbuild_target() {
	TARGET="$1"
	echo "Target set to $TARGET" 1>&2
}

function rbuild_host() {
	local OLD_HOST="$CFG_COMPILER_HOST_TRIPLE"
	export CFG_COMPILER_HOST_TRIPLE="$1"
	echo "Host set to $CFG_COMPILER_HOST_TRIPLE" 1>&2
	if [ "$TARGET" = "$OLD_HOST" ]; then
		rbuild_target "$CFG_COMPILER_HOST_TRIPLE"
	fi
}

function rbuild_target_dir() {
	TARGET_DIR="$1"
}

function rbuild_alloc() {
	case $1 in
		system | jemalloc)
			ALLOC_CRATE="alloc_$1"
			;;
		none)
			ALLOC_CRATE=""
			;;
		*)
			echo "Valid alloc crates: jemalloc system none" 1>&2
			return 1
			;;
	esac
}

function rbuild_profile() {
	case $1 in
		debug | release)
			;;
		*)
			echo "Valid profiles: debug release" 1>&2
			return 1
			;;
	esac

	PROFILE="$1"
}

function rbuild_cargo() {
	COMMAND="$1"
	shift 1

	if [ -z "$CARGO" ]; then
		CARGO="cargo"
	fi

	if [ -z "$CPUCOUNT" ]; then
		CPUCOUNT=`grep -c ^processor /proc/cpuinfo || echo 4`
	fi

	local _CARGO_FLAGS="$CARGO_FLAGS"
	local _RUSTC_FLAGS="$RUSTC_FLAGS"
	_CARGO_FLAGS="$CARGO_FLAGS -j $CPUCOUNT"
	if [ "$PROFILE" == "release" ]; then
		_RUSTC_FLAGS="$_RUSTC_FLAGS -C codegen-units=$CPUCOUNT"
		_CARGO_FLAGS="$_CARGO_FLAGS --release"
	fi

	local RUSTDOC="$RUSTDOC"
	if [ -z "$RUSTDOC" ]; then
		if [ -n "$RBUILD_SYSROOT_BIN" ]; then
			RUSTDOC="$RBUILD_SYSROOT_BIN/bin/rustdoc"
		else
			RUSTDOC=`which rustdoc`
		fi
	fi
	if [ -s "$RUSTDOC" ]; then
		cp -af "$RUSTDOC" "$_RUSTDOC"
	fi

	local RUSTC="$RUSTC"
	if [ -z "$RUSTC" ]; then
		if [ -n "$RBUILD_SYSROOT_BIN" ]; then
			RUSTC="$RBUILD_SYSROOT_BIN/bin/rustc"
		else
			RUSTC=`which rustc`
		fi
	fi

	local RUSTC_NATIVE="$RUSTC_NATIVE"
	if [ -z "$RUSTC_NATIVE" ]; then
		if [ -n "$RBUILD_SYSROOT_NATIVE_BIN" ]; then
			RUSTC_NATIVE="$RBUILD_NATIVE_SYSROOT_BIN/bin/rustc"
		else
			RUSTC_NATIVE=`which rustc`
		fi
	fi

	FEATURES_STD="std $COMPILER_RT"
	if [ -n "$STD_FEATURES" ]; then
		FEATURES_STD="$FEATURES_STD `printf 'std/%s ' $STD_FEATURES`"
	fi
	FEATURES_ALLOC="alloc $ALLOC_CRATE"
	FEATURES_TEST="collectionstest coretest"
	FEATURES_SECONDARY="arena fmt_macros flate getopts graphviz log rbml serialize syntax term test"
	FEATURES_RUSTC="rustc_driver rustdoc"
	FEATURES="$FEATURES_STD $FEATURES_TEST $FEATURES_ALLOC $FEATURES_SECONDARY $FEATURES_RUSTC"

	{
		cat "$ROOT/Cargo.toml.in";
		echo -e '\n[features]\ndefault = ['
		for feature in $FEATURES; do
			echo -e "\\t\"$feature\","
		done
		echo "]"
	} > "$ROOT/Cargo.toml"

	CARGO_TARGET_DIR="$TARGET_DIR" \
	CFG_LLVM_LINKAGE_FILE="$TARGET_DIR/$TARGET/llvmdeps.rs" \
	RUSTC_SYSROOT="$RBUILD_SYSROOT" \
	RUSTC_NATIVE_SYSROOT="$RBUILD_SYSROOT_NATIVE" \
	RUSTC_RUSTC="$RUSTC" \
	RUSTC_NATIVE="$RUSTC_NATIVE" \
	RUSTDOC_RUSTDOC="$_RUSTDOC" \
	RUSTC="$ROOT/rustc-sysroot" \
	RUSTDOC="$ROOT/rustdoc-sysroot" \
	RUSTC_FLAGS="$_RUSTC_FLAGS" \
	"$CARGO" $COMMAND --target "$TARGET" $_CARGO_FLAGS "$@"
}

function rbuild() {
	local cmd="$1"
	shift 1
	case $cmd in
		rustc)
			RUSTC_FLAGS="$RUSTC_FLAGS --cfg rustc" rbuild_cargo build --bin rustc "$@"
			;;
		rustdoc)
			RUSTC_FLAGS="$RUSTC_FLAGS --cfg rustdoc" rbuild_cargo build --bin rustdoc "$@"
			;;
		compiletest | error-index-generator)
			rbuild_cargo build --bin "$1" "$@"
			;;
		std)
			if [ -n "$ALLOC_CRATE" ]; then
				rbuild_cargo build -p "$ALLOC_CRATE" "$@" || return
			fi
			if [ -n "$COMPILER_RT" ]; then
				rbuild_cargo build -p compiler-rt "$@" || return
			fi
			rbuild_cargo build -p std "$@"
			;;
		all)
			if [ -z "$CFG_LLVM_CONFIG" ]; then
				rbuild_llvm_config ||
					rbuild_llvm || return
			fi

			for pkgid in std rustc_driver rustdoc; do
				rbuild_cargo build -p "$pkgid" "$@" || return
			done

			rbuild rustc "$@" &&
			rbuild rustdoc "$@" &&
			rbuild compiletest "$@" &&
			rbuild error-index-generator "$@"
			;;
		*)
			echo "Valid targets: all std rustc rustdoc compiletest error-index-generator" 1>&2
			return 1
			;;
	esac
}

function rbuild_test() {
	rbuild_cargo build -p test "$@" || return

	for crate in "$ROOT/src/"*"/Cargo.toml"; do
		cratename=`basename $(dirname $crate)`
		crate=${cratename#lib}

		case $crate in
			alloc_$ALLOC_CRATE)
				;;
			compiletest | error-index-generator | rustc_driver | rustdoc | alloc_jemalloc | alloc_system)
				continue
				;;
			*)
				;;
		esac

		rbuild_cargo test -p "$crate" "$@" || return
	done
}

function rbuild_gensysroot() {
	local SYSROOT="$1"
	local TYPE=all
	if [ -z "$SYSROOT" ]; then
		echo "Expected path to sysroot" 1>&2
		return 1
	fi

	case "$SYSROOT" in 
		lib | bin)
			TYPE="$SYSROOT"
			SYSROOT="$2"
			;;
	esac

	local SYSROOT_LIB="$SYSROOT/lib/rustlib/$TARGET/lib"
	local SYSROOT_BIN="$SYSROOT/bin"

	if [ "$TYPE" == "all" -o "$TYPE" == "lib" ]; then
		mkdir -p `dirname "$SYSROOT_LIB"`
		rm -f "$SYSROOT_LIB"
		ln -sf "$TARGET_DIR/$TARGET/$PROFILE/deps" "$SYSROOT_LIB"
	fi

	if [ "$TYPE" == "all" -o "$TYPE" == "bin" ]; then
		mkdir -p `dirname "$SYSROOT_BIN"`
		rm -f "$SYSROOT_BIN"
		ln -sf "$TARGET_DIR/$CFG_COMPILER_HOST_TRIPLE/$PROFILE" "$SYSROOT_BIN"
	fi

	echo Sysroot created at "$SYSROOT"
}

function rbuild_sysroot() {
	if [ -z "$1" ]; then
		unset RBUILD_SYSROOT
		unset RBUILD_SYSROOT_BIN
		unset RBUILD_SYSROOT_NATIVE
		unset RBUILD_SYSROOT_NATIVE_BIN
		echo Sysroot cleared
		return
	fi

	local TYPE="$1"
	local SYSROOT="$2"
	if [ "$SYSROOT" == "bin" ]; then
		shift 2
		rbuild_sysroot "$TYPE"-bin "$@"
		SYSROOT="$1"
	fi

	case "$TYPE" in
		all)
			rbuild_sysroot target bin "$SYSROOT" &&
			rbuild_sysroot native bin "$SYSROOT"
			;;
		target)
			RBUILD_SYSROOT="$SYSROOT"
			;;
		target-bin)
			RBUILD_SYSROOT_BIN="$SYSROOT"
			;;
		native)
			RBUILD_SYSROOT_NATIVE="$SYSROOT"
			;;
		native-bin)
			RBUILD_SYSROOT_BIN="$SYSROOT"
			;;
		*)
			echo Expected sysroot type: all target target-bin native native-bin 1>&2
			return 1
			;;
	esac

	echo Sysroot set to "$SYSROOT"
}

function rbuild_stage() {
	local STAGE="$1"
	shift 1

	if [ -z "$STAGE" ]; then
		echo Expected stage name 1>&2
		return 1
	fi

	local STAGE_DIR="$ROOT/$STAGE"
	local STAGE0_DIR="$STAGE_DIR/staging"

	rbuild_target_dir "$STAGE0_DIR" &&
	rbuild rustc "$@" && rbuild rustdoc "$@" &&
	rbuild_gensysroot bin "$STAGE_DIR" &&
	rbuild_sysroot target-bin "$STAGE_DIR" &&
	rbuild_target_dir "$STAGE_DIR" &&
	rbuild std "$@" &&
	TARGET="$CFG_COMPILER_HOST_TRIPLE" rbuild std "$@" &&
	rbuild_gensysroot lib "$STAGE_DIR"
}

if [ -z "$PROFILE" ]; then
	rbuild_profile "debug"
fi
if [ -z "${ALLOC_CRATE+x}" ]; then
	rbuild_alloc "jemalloc"
fi
if [ -z "${COMPILER_RT+x}" ]; then
	COMPILER_RT="compiler-rt"
fi
rbuild_target_dir "$ROOT/target"
rbuild_llvm_config 2> /dev/null || true
rbuild_git_cfg

if [ -z "$_RUSTDOC" ]; then
	_RUSTDOC="`mktemp -d`/rustdoc"
fi
trap 'rm -f "$_RUSTDOC"' EXIT

STD_FEATURES="libc rust_builtin backtrace"

export RUST_TARGET_PATH="$RUST_TARGET_PATH:$ROOT"
export CFG_LLVM_STDCPP="stdc++"
export CFG_LLVM_STDCPP_STATIC="0"
