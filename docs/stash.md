# API and Coordinates Usage
## Important Notes
* UMD accepts Virtual Coordinates (these match Physical Coordinates on Grayskull)
* Any application programming overlay hardware/streams must use Translated Coordinates on a Wormhole Device with translation tables enabled
* Regular Physical Coordinates can be used on a Wormhole Device with Translation Tables disabled
* Translation Tables are a prerequisite for Wormhole Harvesting
* As part of product definition, translation tables are enabled on Wormhole devices regardless of their grid size

### Mapping relationship between NOC0/NOC1
Below is the way we map the physical to `NOC0`/`NOC1` coordinates -- Notice that physical maps to `NOC0`directly and `noc_size_*` changes depending on `ARCH`
```cpp
#define NOC_X(x) (noc == NOC0 ? (x) : (noc_size_x-1-(x)))
#define NOC_Y(y) (noc == NOC0 ? (y) : (noc_size_y-1-(y)))
```

### Important Notes
* These coordinates cannot be used in APIs provided by UMD. UMD expects virtual coordinates to be passed into its APIs (see below for definition).
* This coordinate system is easier to use than physical coordinates and can be used by Buda or Metal. However, certain operations may require the use of physical coordinates. For this, the SOC descriptor class in UMD presents the following translation layers:

```
std::unordered_map<int, int> worker_log_to_routing_x; // worker logical to routing (x)
std::unordered_map<int, int> worker_log_to_routing_y; // worker logical to routing (y)
```

### Important Notes 
* The coordinates are **NOT** contiguous for tensix cores -- In the example shown, ethernet corresponds to physical coords `[*-6]`:  this row is not on in the grid of worker cores
* The coordinates are statically assigned to each "node" regardless of harvesting
* `grayskull` are listed in `[y-x]` while `wormhole_b0` are listed in `[x-y]`