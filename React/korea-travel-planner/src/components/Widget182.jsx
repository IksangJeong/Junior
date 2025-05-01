import React, { useState } from 'react';

/**
 * Widget182 컴포넌트
 */
const Widget182 = ({ title = "위젯 182", items = [] }) => {
    const [expanded, setExpanded] = useState(false);

    return (
        <div className="bg-white rounded-lg shadow p-4 mb-4">
            <div
                className="flex justify-between items-center cursor-pointer"
                onClick={() => setExpanded(!expanded)}
            >
                <h3 className="text-lg font-semibold">{title}</h3>
                <span>{expanded ? '▲' : '▼'}</span>
            </div>
            {expanded && (
                <div className="mt-3">
                    {items.length > 0 ? (
                        <ul className="space-y-2">
                            {items.map((item, idx) => (
                                <li key={idx} className="p-2 bg-gray-50 rounded">
                                    {item}
                                </li>
                            ))}
                        </ul>
                    ) : (
                        <p className="text-gray-400">항목이 없습니다</p>
                    )}
                </div>
            )}
        </div>
    );
};

export default Widget182;
