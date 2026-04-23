#!/usr/bin/env python3
"""HTTPS server para test/ — necessario para acesso a camera no celular."""
import os, ssl, socket, http.server, pathlib

PORT = 8443
DIR  = pathlib.Path(__file__).parent
CERT = DIR / 'cert.pem'
KEY  = DIR / 'key.pem'

def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    finally:
        s.close()

def generate_cert(ip):
    from cryptography import x509
    from cryptography.x509.oid import NameOID
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.hazmat.backends import default_backend
    import datetime, ipaddress

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048,
                                   backend=default_backend())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, ip)])
    cert = (
        x509.CertificateBuilder()
        .subject_name(name).issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(datetime.datetime.utcnow())
        .not_valid_after(datetime.datetime.utcnow() + datetime.timedelta(days=365))
        .add_extension(x509.SubjectAlternativeName([
            x509.IPAddress(ipaddress.IPv4Address(ip)),
            x509.DNSName('localhost'),
        ]), critical=False)
        .sign(key, hashes.SHA256(), default_backend())
    )
    KEY.write_bytes(key.private_bytes(serialization.Encoding.PEM,
                    serialization.PrivateFormat.TraditionalOpenSSL,
                    serialization.NoEncryption()))
    CERT.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    print(f'Certificado gerado para {ip}')

ip = local_ip()

if not (CERT.exists() and KEY.exists()):
    try:
        generate_cert(ip)
    except ImportError:
        print('ERRO: instale a dependencia:  pip install cryptography')
        raise

os.chdir(DIR)

ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain(str(CERT), str(KEY))

httpd = http.server.HTTPServer(('0.0.0.0', PORT), http.server.SimpleHTTPRequestHandler)
httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

print(f'\nServidor HTTPS rodando.')
print(f'  Abra no celular: https://{ip}:{PORT}/observer.html')
print(f'\nAo abrir, aceite o aviso de "conexao nao segura" (certificado autoassinado).')
print('Ctrl+C para parar.\n')
httpd.serve_forever()
