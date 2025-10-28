#!/usr/bin/env bash

update-alternatives-many() {
  local exe="$1" version="$2" priority="$3"
  shift 3
  local args=(--install "/usr/bin/${exe}" "${exe}" "/usr/bin/${exe}-${version}" "${priority}")
  local slave slave_tgt
  for slave in "$@"; do
    if [[ "$slave" = *.py ]]; then
      slave_tgt="${slave%.py}-${version}.py"
    else
      slave_tgt="${slave}-${version}"
    fi
    args+=(--slave "/usr/bin/${slave}" "${slave}" "/usr/bin/${slave_tgt}")
  done
  update-alternatives "${args[@]}"
}

update-alternatives-llvm() {
  local version="$1" priority="${2:-100}"

  update-alternatives-many clang "${version}" "${priority}" \
    clang++ clang-cpp asan_symbolize

  update-alternatives-many clang-format "${version}" "${priority}" \
    clang-format-diff git-clang-format

  update-alternatives-many clang-tidy "${version}" "${priority}" \
    clang-tidy-diff.py run-clang-tidy run-clang-tidy.py

  # clang-tools
  update-alternatives-many clang-check "${version}" "${priority}" \
    amdgpu-arch analyze-build c-index-test clang-apply-replacements \
    clang-change-namespace clang-cl clang-doc clang-extdef-mapping \
    clang-include-cleaner clang-include-fixer clang-installapi \
    clang-linker-wrapper clang-move clang-nvlink-wrapper \
    clang-offload-bundler clang-offload-packager \
    clang-query clang-refactor clang-reorder-fields \
    clang-repl clang-scan-deps clang-tblgen diagtool find-all-symbols \
    hmaptool hwasan_symbolize intercept-build modularize nvptx-arch \
    pp-trace sancov scan-build scan-build-py scan-view

  update-alternatives-many clangd "${version}" "${priority}"

  update-alternatives-many lld "${version}" "${priority}" \
    ld.lld ld64.lld lld-link wasm-ld

  update-alternatives-many lldb "${version}" "${priority}" \
    lldb-argdumper lldb-dap lldb-instr lldb-server

  # llvm-runtime
  update-alternatives-many lli "${version}" "${priority}" \
    lli-child-target

  # llvm-tools
  update-alternatives-many not "${version}" "${priority}" \
    FileCheck UnicodeNameMappingGenerator count \
    split-file yaml-bench

  # llvm
  update-alternatives-many llvm-config "${version}" "${priority}" \
    bugpoint dsymutil llc llvm-PerfectShuffle llvm-addr2line \
    llvm-ar llvm-as llvm-bcanalyzer llvm-bitcode-strip llvm-c-test \
    llvm-cat llvm-cfi-verify llvm-cov llvm-cvtres llvm-cxxdump \
    llvm-cxxfilt llvm-cxxmap llvm-debuginfo-analyzer llvm-debuginfod \
    llvm-debuginfod-find llvm-diff llvm-dis llvm-dlltool llvm-dwarfdump \
    llvm-dwarfutil llvm-dwp llvm-exegesis llvm-extract llvm-gsymutil \
    llvm-ifs llvm-install-name-tool llvm-jitlink llvm-jitlink-executor \
    llvm-lib llvm-libtool-darwin llvm-link llvm-lipo llvm-lto llvm-lto2 \
    llvm-mc llvm-mca llvm-ml llvm-modextract llvm-mt llvm-nm llvm-objcopy \
    llvm-objdump llvm-opt-report llvm-otool llvm-pdbutil llvm-profdata \
    llvm-profgen llvm-ranlib llvm-rc llvm-readelf llvm-readobj \
    llvm-readtapi llvm-reduce llvm-remarkutil llvm-rtdyld llvm-sim \
    llvm-size llvm-split llvm-stress llvm-strings llvm-strip \
    llvm-symbolizer llvm-tblgen llvm-tli-checker llvm-undname \
    llvm-windres llvm-xray obj2yaml opt reduce-chunk-list sanstats \
    verify-uselistorder yaml2obj

}

update-alternatives-llvm "$1" "$2"
