"""WiFi station connection with simple retry."""

import network

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio


async def connect(cfg):
    """Bring up the station interface and connect using the ``wifi`` config.

    Waits up to ~20 s for an association.

    Parameters
    ----------
    cfg : config.Config
        Shared config; the ``wifi`` section supplies ssid/password/hostname.

    Returns
    -------
    network.WLAN
        The station interface, connected or not (check ``isconnected()``).
    """
    ssid     = cfg.get("wifi.ssid")
    password = cfg.get("wifi.password")
    hostname = cfg.get("wifi.hostname", "esp32-robot")

    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    try:
        sta.config(dhcp_hostname=hostname)
    except Exception:
        pass

    if not sta.isconnected():
        print("WiFi: connecting to", ssid)
        sta.connect(ssid, password)
        for _ in range(40):  # ~20 s
            if sta.isconnected():
                break
            await asyncio.sleep_ms(500)

    if sta.isconnected():
        print("WiFi: connected", sta.ifconfig())
    else:
        print("WiFi: FAILED to connect")
    return sta


async def keepalive_task(cfg, sta):
    """Reconnect the station whenever the link drops (polls every 5 s).

    Parameters
    ----------
    cfg : config.Config
        Shared config supplying the credentials for reconnection.
    sta : network.WLAN
        The station interface returned by :func:`connect`.

    Returns
    -------
    None
        Never returns (runs for the program lifetime).
    """
    while True:
        if not sta.isconnected():
            print("WiFi: link lost, reconnecting...")
            try:
                sta.connect(cfg.get("wifi.ssid"), cfg.get("wifi.password"))
            except Exception as e:
                print("WiFi reconnect error:", e)
        await asyncio.sleep(5)
