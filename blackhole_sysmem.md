# Instructions for setting up sysmem for Blackhole:

* Ensure system IOMMU is enabled.  You may need to remove `iommu=pt` from your /etc/default/grub and then run `sudo grub-update` and reboot.

    Check `01:00.0 Processing accelerators: Tenstorrent Inc Blackhole` for IOMMU support:
    ```
    cat /sys/bus/pci/devices/0000\:01\:00.0/iommu_group/type
    ```
    and look for `DMA` or `DMA-FQ`.

* Check out, build, and load TT-KMD with Blackhole and IOMMU support.
    ```
    git clone git@yyz-gitlab.local.tenstorrent.com:syseng-platform/tt-kmd.git
    cd tt-kmd
    git checkout joelsmith/alewycky-iommu-bh  # this branch is alewycky/iommu rebased on blackhole
    make
    sudo rmmod tenstorrent
    sudo insmod ./tenstorrent.ko dma_address_bits=56
    ```

    Suggestion: run `dmesg` and see that TT-KMD noticed the Blackhole card(s).

* Check out and build TT-UMD with Blackhole and IOMMU support.
    ```
    git clone git@github.com:tenstorrent/tt-umd.git
    cd tt-umd
    git checkout joelsmith/bh
    git submodule update --init
    mkdir my-cmake-build
    cd my-cmake-build
    cmake .. && make -j
    ./bh-test
    ```

    You should see something like
    ```
    2024-06-17 16:21:15.620 | INFO     | SiliconDriver   - Detected 1 PCI device : {0}
    ---- ttSiliconDevice::init_iommu_allocations: physical_device_id: 0 ch: 0 mapping: 0x7f110d900000 IOVA: 0xffffff80000000
    ---- ttSiliconDevice::init_iommu_allocations: physical_device_id: 0 ch: 1 mapping: 0x7f114d900000 IOVA: 0xffffffc0000000
    Sysmem test passed (2097152 bytes)
    ```

### Notes
* IOMMU is necessary for this to work
* This does not (but could) use 1G huge pages 