"""
데이터베이스 연습 #316
SQLite CRUD 실습
"""
import sqlite3

def setup_316():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_316 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_316_0', 225.1, 'A'), ('item_316_1', 148.73, 'B'), ('item_316_2', 241.82, 'C'), ('item_316_3', 580.35, 'C'), ('item_316_4', 601.94, 'C'), ('item_316_5', 820.2, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_316 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_316()
    cursor = conn.cursor()

    print(f"DB 연습 #316")
    cursor.execute('SELECT * FROM items_316')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_316 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
