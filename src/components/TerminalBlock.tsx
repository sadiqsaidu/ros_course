import { Terminal } from 'lucide-react';

interface TerminalBlockProps {
  command: string;
  output?: string;
  title?: string;
}

export default function TerminalBlock({ command, output, title = "Terminal" }: TerminalBlockProps) {
  return (
    <div className="my-8 rounded-xl overflow-hidden border border-zinc-800 bg-zinc-900 text-zinc-100 shadow-2xl">
      <div className="flex items-center justify-between px-4 py-2 bg-zinc-800/50 border-b border-zinc-700/50">
        <span className="text-xs font-mono text-zinc-400 flex items-center gap-2">
          <Terminal size={12} /> {title}
        </span>
      </div>
      <div className="p-6 font-mono text-sm space-y-3">
        <div className="flex space-x-3">
          <span className="text-green-400 font-bold select-none">➜</span>
          <span className="flex-1 text-zinc-100">{command}</span>
        </div>
        {output && (
          <div className="text-zinc-400 pl-6 border-l-2 border-zinc-700/50 py-1">
            {output.split('\n').map((line, i) => (
              <div key={i}>{line}</div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
