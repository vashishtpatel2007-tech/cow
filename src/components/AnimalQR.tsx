/**
 * §9.4 — generate a printable QR resolving to /a/:public_slug.
 *
 * This is the half of the product that scales (§9.7): it works on animals that
 * will never wear a collar, needs no app on the scanner's phone, and costs
 * nothing per animal beyond the ink.
 *
 * Printed on an ear tag it lives outdoors for years, so: high error correction
 * (level H tolerates ~30% damage — mud, sun-bleaching, a torn corner), a wide
 * quiet zone, and the tag number printed in human-readable text underneath as
 * the fallback when the code is finally too scuffed to scan.
 *
 * This will be the tag that connects every cow to its farmer to help us connect farmer to cattle in case of emergency.
 */

import { useEffect, useRef, useState } from 'react';
import QRCode from 'qrcode';

interface Props {
  slug: string;
  animalName: string;
  tagNumber: string | null;
  size?: number;
}

export function AnimalQR({ slug, animalName, tagNumber, size = 256 }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [dataUrl, setDataUrl] = useState<string | null>(null);
  const url = `${window.location.origin}/a/${slug}`;

  useEffect(() => {
    if (!canvasRef.current) return;
    void QRCode.toCanvas(canvasRef.current, url, {
      width: size,
      margin: 3,               // wide quiet zone survives a trimmed print
      errorCorrectionLevel: 'H',
      color: { dark: '#10231A', light: '#FFFFFF' },
    }).then(() => setDataUrl(canvasRef.current?.toDataURL('image/png') ?? null));
  }, [url, size]);

  function print() {
    const w = window.open('', '_blank');
    if (!w || !dataUrl) return;
    // Self-contained document: a farmer printing at a village shop will not
    // have this app's stylesheet available on that machine.
    w.document.write(`
      <html><head><title>${animalName}</title>
      <style>
        @page { margin: 12mm; }
        body { font-family: system-ui, sans-serif; text-align: center; padding: 24px; }
        img { width: 62mm; height: 62mm; }
        h1 { font-size: 22pt; margin: 10px 0 2px; }
        p  { font-size: 13pt; margin: 2px 0; }
        .tag { font-size: 17pt; font-weight: 700; letter-spacing: 1px; }
      </style></head>
      <body>
        <img src="${dataUrl}" alt="" />
        <h1>${animalName}</h1>
        <p class="tag">${tagNumber ?? ''}</p>
        <p>Scan to contact the owner</p>
      </body></html>`);
    w.document.close();
    w.focus();
    w.print();
  }

  return (
    <div className="card text-center">
      <canvas ref={canvasRef} className="mx-auto" />
      <p className="tnum mt-2 text-[16px] font-bold">{tagNumber}</p>
      <p className="mt-1 break-all text-[13px] font-semibold opacity-60">{url}</p>
      <button onClick={print} className="btn btn-primary mt-3 w-full">Print tag</button>
    </div>
  );
}
