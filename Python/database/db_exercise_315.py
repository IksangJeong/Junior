"""
데이터베이스 연습 #315
SQLite CRUD 실습
"""
import sqlite3

def setup_315():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_315 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_315_0', 768.39, 'A'), ('item_315_1', 248.26, 'A'), ('item_315_2', 109.89, 'A'), ('item_315_3', 451.98, 'C'), ('item_315_4', 706.25, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_315 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_315()
    cursor = conn.cursor()

    print(f"DB 연습 #315")
    cursor.execute('SELECT * FROM items_315')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_315 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
