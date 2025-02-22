

if __name__ == "__main__":
    drone_ips = ["192.168.10.2", "192.168.10.3"]
    server_ip = "192.168.10.4"
    base_port = 10000

    proxy_server = VideoProxyServer(drone_ips, server_ip, base_port)
    proxy_server.start_proxy()
