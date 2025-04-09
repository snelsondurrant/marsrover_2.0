#!/usr/bin/env python3
# Created by Nelson Durrant (w Google Gemini 2.5 Pro), Mar 2025
import requests
import mercantile
import time
import argparse
import sys


def get_tile_coords_for_bounds(bounds, zoom):
    """
    Converts geographic bounds (N,S,E,W dict) to tile coordinate ranges (min/max X/Y dict).
    """

    try:
        nw_tile = mercantile.tile(bounds["west"], bounds["north"], zoom)
        se_tile = mercantile.tile(bounds["east"], bounds["south"], zoom)
        min_x, max_x = min(nw_tile.x, se_tile.x), max(nw_tile.x, se_tile.x)
        min_y, max_y = min(nw_tile.y, se_tile.y), max(nw_tile.y, se_tile.y)
        return {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y}
    except Exception as e:
        print(f"Error converting bounds to tiles: {e}", file=sys.stderr)
        return None


def cache_tiles_in_range(tile_range, zoom, base_url):
    """
    Downloads tiles within the specified range from MapProxy.
    """

    if not tile_range:
        print("Error: Invalid tile range provided.", file=sys.stderr)
        return False  # Indicate failure

    min_x, max_x = tile_range["min_x"], tile_range["max_x"]
    min_y, max_y = tile_range["min_y"], tile_range["max_y"]

    total_tiles = (max_x - min_x + 1) * (max_y - min_y + 1)
    if total_tiles <= 0:
        print("Warning: Calculated tile range is empty. No tiles to cache.")
        return True  # Not an error, just nothing to do

    print(
        f"Caching level {zoom}: X={min_x}..{max_x}, Y={min_y}..{max_y} ({total_tiles} tiles)"
    )

    count = 0
    errors = 0
    start_time = time.time()
    session = requests.Session()  # Use a session for potential connection reuse

    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            count += 1
            tile_url = f"{base_url}/{zoom}/{x}/{y}.png"
            attempt = 0
            success = False
            retries = 2  # Try original + 2 retries

            while attempt <= retries and not success:
                try:
                    response = session.get(tile_url, timeout=15)  # Network timeout
                    if response.status_code == 200:
                        success = True
                    # Don't retry client errors (4xx), only server errors (5xx) or timeouts
                    elif response.status_code < 500:
                        if attempt == 0:
                            print(
                                f"Warning: Status {response.status_code} for {tile_url}",
                                file=sys.stderr,
                            )
                        break  # Don't retry client errors
                    else:  # Server error (5xx)
                        if attempt == 0:
                            print(
                                f"Warning: Status {response.status_code} for {tile_url}",
                                file=sys.stderr,
                            )

                except requests.exceptions.Timeout:
                    if attempt == 0:
                        print(
                            f"Warning: Timeout requesting {tile_url}", file=sys.stderr
                        )
                except requests.exceptions.RequestException as e:
                    if attempt == 0:
                        print(
                            f"Warning: Request Exception for {tile_url}: {e}",
                            file=sys.stderr,
                        )
                    # Don't retry persistent connection errors immediately
                    if attempt < retries:
                        time.sleep(1)  # Small delay before retry
                    else:
                        break

                if not success and attempt < retries:
                    time.sleep(1 + attempt)  # Basic exponential backoff delay
                attempt += 1

            if not success:
                errors += 1

            # Print progress update periodically
            if count % 100 == 0 or count == total_tiles:
                elapsed = time.time() - start_time
                print(
                    f"  Progress: {count}/{total_tiles} processed ({errors} errors). Elapsed: {elapsed:.1f}s"
                )

    print("\nCaching finished.")
    print(f"Tiles processed: {total_tiles}")
    print(f"Errors encountered (after retries): {errors}")
    return errors == 0  # True if no errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Download MapProxy tiles for a specified bounding box and zoom level."
    )

    # Required arguments
    parser.add_argument(
        "--north", type=float, required=True, help="North latitude boundary."
    )
    parser.add_argument(
        "--south", type=float, required=True, help="South latitude boundary."
    )
    parser.add_argument(
        "--east", type=float, required=True, help="East longitude boundary."
    )
    parser.add_argument(
        "--west", type=float, required=True, help="West longitude boundary."
    )

    # Optional arguments with defaults
    parser.add_argument(
        "--level", type=int, default=19, help="Zoom level to cache (default: 19)."
    )
    parser.add_argument(
        "--url",
        type=str,
        default="http://localhost:8080/wmts/gm_layer/gm_grid",
        help="Base URL of the MapProxy WMTS grid layer.",
    )

    args = parser.parse_args()

    # Input validation
    if args.north <= args.south:
        print(
            "Error: North latitude must be greater than South latitude.",
            file=sys.stderr,
        )
        sys.exit(1)
    if args.east <= args.west:  # Basic check (doesn't handle anti-meridian)
        print(
            "Error: East longitude must be greater than West longitude.",
            file=sys.stderr,
        )
        sys.exit(1)
    if not (0 <= args.level <= 22):
        print(
            f"Error: Zoom level {args.level} is invalid (must be 0-22).",
            file=sys.stderr,
        )
        sys.exit(1)

    user_bounds = {
        "north": args.north,
        "south": args.south,
        "east": args.east,
        "west": args.west,
    }
    print(
        f"Input Bounds: N={user_bounds['north']}, S={user_bounds['south']}, W={user_bounds['west']}, E={user_bounds['east']}"
    )

    # Convert to tile coordinates
    tile_bounds = get_tile_coords_for_bounds(user_bounds, args.level)

    if tile_bounds:
        # Run caching process
        success = cache_tiles_in_range(tile_bounds, args.level, args.url)
        if success:
            print("Script finished successfully.")
            sys.exit(0)  # Exit code 0 for success
        else:
            print("Script finished with errors.", file=sys.stderr)
            sys.exit(1)  # Exit code 1 for failure (any error occurred)
    else:
        print("Could not determine tile boundaries. Exiting.", file=sys.stderr)
        sys.exit(1)
