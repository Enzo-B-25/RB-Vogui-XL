from commande_modif import Commande

# Simule la réception d'une commande
cmd = Commande()
cmd.SetStartStream('192.168.0.133', 5001, 0)  # IP du Battle Center, port, caméra /dev/video0

print("[Robot] Reçu commande de type :", cmd.getTypeCommande())
cmd.StartStream()
